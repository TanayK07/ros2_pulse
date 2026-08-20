"""pulse-top — live TUI over the ros2_pulse probe's jsonl log.

Pure consumer: tails the file the probe already writes (ROS_TOPIC_STATS_FORMAT=jsonl),
parses each window, renders. No ROS dependency, no graph presence, zero cost to the
probed system beyond the file read — works over ssh and on dead logs after the fact,
which is exactly what the graph-joining monitors (ros2top, ornis, ...) cannot do.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import tempfile

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, Static, TabbedContent, TabPane, Tree

from .model import StatsState, parse_jsonl_line, sparkline
from .reader import FileFollower

ACCENT = "#b48cf2"
GOOD = "#7ee2a8"
WARN = "#f2c96b"
BAD = "#f27d72"
DIM = "#7c8797"

COLUMNS = ("TOPIC", "PUB Hz", "INTRA", "RECV", "GAP ms", "60s")
SORTS = ("topic", "rate", "gap")


def fmt(v: float | None, none: str = "—") -> str:
    return none if v is None else f"{v:.1f}"


class PulseTopApp(App):
    CSS_PATH = "app.tcss"
    BINDINGS = [
        Binding("q", "quit", "quit"),
        Binding("s", "cycle_sort", "sort"),
        Binding("w", "toggle_warns_only", "warns only"),
    ]

    def __init__(self, path: str, poll_s: float = 0.5):
        super().__init__()
        self._path = path
        self._poll_s = poll_s
        self._follower = FileFollower(path)
        self._state = StatsState()
        self._sort = 0
        self._warns_only = False
        self._selected: str | None = None

    def compose(self) -> ComposeResult:
        yield Static(id="topbar")
        with TabbedContent(initial="topics"):
            with TabPane("Topics", id="topics"):
                with Horizontal():
                    with Vertical(id="table-col"):
                        yield DataTable(id="table", cursor_type="row")
                        yield Static(id="warns-strip")
                    yield Static(id="sidebar")
            with TabPane("Tree", id="tree"):
                yield Tree("/", id="ns-tree")
            with TabPane("Nodes", id="nodes"):
                yield Static(id="nodes-list")
            with TabPane("Warns", id="warns"):
                yield Static(id="warns-list")
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one("#table", DataTable)
        for col in COLUMNS:
            table.add_column(col, key=col)
        self._refresh_topbar()
        self.set_interval(self._poll_s, self._tick)

    def _tick(self) -> None:
        changed = False
        for line in self._follower.poll():
            w = parse_jsonl_line(line)
            if w is not None:
                self._state.apply(w)
                changed = True
        if changed:
            self._refresh_all()

    # ---- rendering ----

    def _topic_rows(self):
        items = list(self._state.topics.items())
        if self._warns_only:
            items = [it for it in items if it[1].warn_kind]
        key = SORTS[self._sort]
        if key == "topic":
            items.sort(key=lambda it: it[0])
        elif key == "rate":
            items.sort(key=lambda it: -it[1].rate)
        else:
            items.sort(key=lambda it: -(it[1].latest.pub_max_dt_ms or it[1].latest.recv_max_dt_ms or 0.0))
        return items

    def _refresh_all(self) -> None:
        self._refresh_topbar()
        self._refresh_table()
        self._refresh_warns_strip()
        self._refresh_sidebar()
        self._refresh_tree()
        self._refresh_nodes()
        self._refresh_warns_tab()

    def _refresh_topbar(self) -> None:
        s = self._state
        n = len(s.warns)
        warn_part = f"[bold {WARN}]{n} warns[/]" if n else f"[{DIM}]0 warns[/]"
        self.query_one("#topbar", Static).update(
            f"[bold {ACCENT}]pulse-top[/] [{DIM}]{self._path} · "
            f"window {s.window_s:.1f}s · {s.windows_seen} seen[/]  {warn_part}"
        )

    def _row_cells(self, name: str, st) -> tuple:
        t = st.latest
        color = BAD if st.warn_kind in ("topic_gap", "node_missing") else WARN if st.warn_kind else None
        gap = t.pub_max_dt_ms if t.pub_max_dt_ms is not None else t.recv_max_dt_ms
        return (
            Text(name, style=f"bold {color}" if color else ""),
            Text(fmt(t.pub_inter_hz), justify="right"),
            Text(fmt(t.pub_intra_hz), justify="right", style=GOOD if t.pub_intra_hz else DIM),
            Text(fmt(t.recv_inter_hz), justify="right"),
            Text(fmt(gap), justify="right", style=f"bold {BAD}" if st.warn_kind == "topic_gap" else DIM),
            Text(sparkline(st.rate_history), style=color or DIM),
        )

    def _refresh_table(self) -> None:
        table = self.query_one("#table", DataTable)
        rows = self._topic_rows()
        want = [name for name, _ in rows]
        have = [rk.value for rk in table.rows]
        if want != have:
            table.clear()
            for name, st in rows:
                table.add_row(*self._row_cells(name, st), key=name)
        else:
            for name, st in rows:
                for col, cell in zip(COLUMNS, self._row_cells(name, st)):
                    table.update_cell(name, col, cell)
        if self._selected is None and want:
            self._selected = want[0]

    def _refresh_warns_strip(self) -> None:
        lines = [f"[{DIM} bold]WARNS · structured from jsonl[/]"]
        for w in self._state.warns:
            c = WARN if w.kind == "topic_rate" else BAD
            lines.append(f"[bold {c}]{w.kind}[/] {w.detail}")
        if not self._state.warns:
            lines.append(f"[{DIM}]none[/]")
        self.query_one("#warns-strip", Static).update("\n".join(lines))

    def _refresh_sidebar(self) -> None:
        sb = self.query_one("#sidebar", Static)
        st = self._state.topics.get(self._selected or "")
        if st is None:
            sb.update(f"[{DIM}]SELECTED\n(no topic)[/]")
            return
        t = st.latest
        spark = sparkline(st.rate_history, width=24)
        color = BAD if st.warn_kind else ACCENT
        sb.update(
            f"[{DIM} bold]SELECTED[/]\n"
            f"[bold {color}]{self._selected}[/]\n\n"
            f"[{color}]{spark}[/]\n\n"
            f"[{DIM}]pub inter[/]  {fmt(t.pub_inter_hz)} Hz\n"
            f"[{DIM}]pub intra[/]  {fmt(t.pub_intra_hz)} Hz\n"
            f"[{DIM}]recv inter[/] {fmt(t.recv_inter_hz)} Hz\n"
            f"[{DIM}]recv intra[/] {fmt(t.recv_intra_hz)} Hz\n"
            f"[{DIM}]pub gap[/]    {fmt(t.pub_max_dt_ms)} ms\n"
            f"[{DIM}]recv gap[/]   {fmt(t.recv_max_dt_ms)} ms\n"
            f"[{DIM}]endpoint[/]   {'seen' if t.recv_endpoint_seen else '—'}"
        )

    def _refresh_tree(self) -> None:
        tree = self.query_one("#ns-tree", Tree)
        tree.clear()
        nodes = {"": tree.root}
        for name in sorted(self._state.topics):
            st = self._state.topics[name]
            parts = name.strip("/").split("/")
            path = ""
            for i, part in enumerate(parts):
                parent = nodes[path]
                path = f"{path}/{part}"
                if path not in nodes:
                    if i == len(parts) - 1:
                        color = BAD if st.warn_kind == "topic_gap" else WARN if st.warn_kind else DIM
                        label = Text.from_markup(f"{part} [{color}]{st.rate:.1f}[/]")
                        nodes[path] = parent.add_leaf(label)
                    else:
                        nodes[path] = parent.add(part, expand=True)
        tree.root.expand()

    def _refresh_nodes(self) -> None:
        lines = []
        for name, alive in sorted(self._state.nodes.items()):
            mark = f"[{GOOD}]■[/]" if alive else f"[{BAD}]□[/]"
            style = "" if alive else f" [{BAD}]missing[/]"
            lines.append(f"{mark} {name}{style}")
        self.query_one("#nodes-list", Static).update("\n".join(lines) or f"[{DIM}]no NODE lines yet[/]")

    def _refresh_warns_tab(self) -> None:
        lines = []
        for w in self._state.warns:
            c = WARN if w.kind == "topic_rate" else BAD
            lines.append(f"[bold {c}]{w.kind:<14}[/] {w.detail}")
        self.query_one("#warns-list", Static).update("\n".join(lines) or f"[{DIM}]no active warns[/]")

    # ---- interaction ----

    def on_data_table_row_highlighted(self, event: DataTable.RowHighlighted) -> None:
        if event.row_key is not None and event.row_key.value:
            self._selected = event.row_key.value
            self._refresh_sidebar()

    def action_cycle_sort(self) -> None:
        self._sort = (self._sort + 1) % len(SORTS)
        self.notify(f"sort: {SORTS[self._sort]}", timeout=1.5)
        self._refresh_table()

    def action_toggle_warns_only(self) -> None:
        self._warns_only = not self._warns_only
        self.notify(f"warns only: {'on' if self._warns_only else 'off'}", timeout=1.5)
        self._refresh_table()


def default_log_path() -> str | None:
    tmp = os.environ.get("TMPDIR") or tempfile.gettempdir()
    candidates = glob.glob(os.path.join(tmp, "topic_freq.*.log")) + glob.glob(
        os.path.join("/tmp", "topic_freq.*.log")
    )
    return max(candidates, key=os.path.getmtime) if candidates else None


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="pulse-top",
        description="Live TUI over a ros2_pulse jsonl log (run the probe with ROS_TOPIC_STATS_FORMAT=jsonl).",
    )
    ap.add_argument("file", nargs="?", help="probe log (default: newest $TMPDIR/topic_freq.<pid>.log)")
    ap.add_argument("--demo", action="store_true", help="run against a self-generated demo log")
    ap.add_argument("--poll", type=float, default=0.5, help="file poll interval seconds (default 0.5)")
    args = ap.parse_args()

    if args.demo:
        from .demo import start_demo_writer

        path = start_demo_writer()
    else:
        path = args.file or default_log_path()
        if path is None:
            print(
                "pulse-top: no probe log found. Start the probe with "
                "ROS_TOPIC_STATS_FORMAT=jsonl, pass a path, or try --demo.",
                file=sys.stderr,
            )
            return 2

    PulseTopApp(path, poll_s=args.poll).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
