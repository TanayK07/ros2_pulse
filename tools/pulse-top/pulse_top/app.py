"""pulse-top, live TUI over the ros2_pulse probe's jsonl log.

Pure consumer: tails the file the probe already writes (ROS_TOPIC_STATS_FORMAT=jsonl),
parses each window, renders. No ROS dependency, no graph presence, zero cost to the
probed system beyond the file read, works over ssh and on dead logs after the fact,
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
from .reader import MultiFollower

ACCENT = "#b48cf2"
GOOD = "#7ee2a8"
WARN = "#f2c96b"
BAD = "#f27d72"
DIM = "#7c8797"

COLUMNS = ("TOPIC", "PUB Hz", "INTRA", "RECV", "GAP ms", "60s")
SORTS = ("topic", "rate", "gap")


def fmt_age(seconds: float) -> str:
    """Human age for a stale row: 12s, 3m, 2h, seconds granularity only under a minute,
    so a row's label (and hence its cell) changes at most once a second."""
    s = int(seconds)
    if s < 60:
        return f"{s}s"
    if s < 3600:
        return f"{s // 60}m"
    return f"{s // 3600}h"


def fmt(v: float | None, none: str = "—") -> str:
    return none if v is None else f"{v:.1f}"


class PulseTopApp(App):
    TITLE = "pulse-top"
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
        self._follower = MultiFollower(path)
        self._state = StatsState()
        # Last-rendered (plain, style) per cell. DataTable.update_cell invalidates
        # the row render caches and schedules a refresh even when the value is
        # identical (Textual 8.2), so only cells whose text or style actually
        # changed are pushed, 240 no-op updates a tick repainted the whole table
        # twice a second over ssh (Orin, 2026-08-23).
        self._rendered: dict[tuple[str, str], tuple[str, str]] = {}
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
            # live rows by rate, then stale rows by rate, a stale 20 Hz topic
            # must not sit above a live 5 Hz one.
            items.sort(key=lambda it: (self._state.is_stale(it[0]), -it[1].rate))
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
            f"{len(self._follower.files)} file(s) · window {s.window_s:.1f}s · "
            f"{s.windows_seen} windows[/]  {warn_part}"
        )

    def _row_cells(self, name: str, st) -> tuple:
        t = st.latest
        # The probe omits silent topics per window: absence is "no traffic seen",
        # not "still at the old rate". Render stale rows as stale, never repeat
        # the last measurement as if current (PR #32 review). Age is wall time
        # from the window timestamps, windows from every process interleave.
        if self._state.is_stale(name):
            stale = Text("—", justify="right", style=DIM)
            return (
                Text(f"{name}  · stale {fmt_age(self._state.age_s(name))}", style=DIM),
                stale, stale.copy(), stale.copy(), stale.copy(),
                Text(sparkline(st.rate_history), style=DIM),
            )
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
            self._rendered.clear()
            for name, st in rows:
                cells = self._row_cells(name, st)
                table.add_row(*cells, key=name)
                for col, cell in zip(COLUMNS, cells):
                    self._rendered[(name, col)] = (cell.plain, str(cell.style))
        else:
            for name, st in rows:
                for col, cell in zip(COLUMNS, self._row_cells(name, st)):
                    sig = (cell.plain, str(cell.style))
                    if self._rendered.get((name, col)) != sig:
                        self._rendered[(name, col)] = sig
                        table.update_cell(name, col, cell)
        if self._selected is None and want:
            self._selected = want[0]

    def _refresh_warns_strip(self) -> None:
        # Log-derived strings (warn details carry topic/node names) are rendered
        # as Text objects, never through a markup parser, a corrupt or hostile
        # log must not be able to crash or restyle the viewer built to inspect
        # it (PR #32 review; Textual's own markup parser makes escape()-based
        # fixes version-fragile, Text assembly is parser-proof).
        out = Text()
        out.append("WARNS · structured from jsonl", style=f"{DIM} bold")
        for w in self._state.warns:
            c = WARN if w.kind == "topic_rate" else BAD
            out.append("\n").append(w.kind, style=f"bold {c}").append(" ").append(w.detail)
        if not self._state.warns:
            out.append("\n").append("none", style=DIM)
        self.query_one("#warns-strip", Static).update(out)

    def _refresh_sidebar(self) -> None:
        sb = self.query_one("#sidebar", Static)
        st = self._state.topics.get(self._selected or "")
        if st is None:
            sb.update(f"[{DIM}]SELECTED\n(no topic)[/]")
            return
        t = st.latest
        spark = sparkline(st.rate_history, width=24)
        color = BAD if st.warn_kind else ACCENT
        rows = [
            ("pub inter ", f"{fmt(t.pub_inter_hz)} Hz"),
            ("pub intra ", f"{fmt(t.pub_intra_hz)} Hz"),
            ("recv inter", f"{fmt(t.recv_inter_hz)} Hz"),
            ("recv intra", f"{fmt(t.recv_intra_hz)} Hz"),
            ("pub gap   ", f"{fmt(t.pub_max_dt_ms)} ms"),
            ("recv gap  ", f"{fmt(t.recv_max_dt_ms)} ms"),
            ("endpoint  ", "seen" if t.recv_endpoint_seen else "—"),
        ]
        out = Text()
        out.append("SELECTED", style=f"{DIM} bold").append("\n")
        out.append(self._selected or "", style=f"bold {color}")
        out.append("\n\n").append(spark, style=color).append("\n")
        for label, value in rows:
            out.append("\n").append(label, style=DIM).append(" ").append(value)
        sb.update(out)

    def _leaf_label(self, part: str, st) -> Text:
        # Text.assemble, not markup: namespace parts are log-derived (PR #32 review).
        color = BAD if st.warn_kind == "topic_gap" else WARN if st.warn_kind else DIM
        return Text.assemble(part, " ", (f"{st.rate:.1f}", color))

    def _refresh_tree(self) -> None:
        # Rates change every window; the topic SET changes on topology events only.
        # Relabel leaves in place on the common path so the user's cursor, expand
        # state and scroll survive live updates; rebuild only when the topology
        # actually changed (PR #32 review).
        tree = self.query_one("#ns-tree", Tree)
        names = sorted(self._state.topics)
        if names == getattr(self, "_tree_names", None):
            for name in names:
                leaf = self._tree_leaves[name]
                part = name.strip("/").split("/")[-1]
                leaf.set_label(self._leaf_label(part, self._state.topics[name]))
            return
        tree.clear()
        self._tree_names = names
        self._tree_leaves = {}
        nodes = {"": tree.root}
        for name in names:
            st = self._state.topics[name]
            parts = name.strip("/").split("/")
            path = ""
            for i, part in enumerate(parts):
                parent = nodes[path]
                path = f"{path}/{part}"
                if path not in nodes:
                    if i == len(parts) - 1:
                        nodes[path] = parent.add_leaf(self._leaf_label(part, st))
                        self._tree_leaves[name] = nodes[path]
                    else:
                        nodes[path] = parent.add(part, expand=True)
        tree.root.expand()

    def _refresh_nodes(self) -> None:
        out = Text()
        for i, (name, alive) in enumerate(sorted(self._state.nodes.items())):
            if i:
                out.append("\n")
            out.append("■ " if alive else "□ ", style=GOOD if alive else BAD)
            out.append(name)
            if not alive:
                out.append(" missing", style=BAD)
        self.query_one("#nodes-list", Static).update(
            out if out.plain else Text("no NODE lines yet", style=DIM)
        )

    def _refresh_warns_tab(self) -> None:
        # Retained view: a one-window transient stays readable with its age
        # instead of blinking for one window period (PR #32 review). Live warns
        # bold; historical ones dimmed with "Nw ago".
        out = Text()
        for i, (fired, w) in enumerate(reversed(self._state.recent_warns)):
            if i:
                out.append("\n")
            age = self._state.warn_age_s(fired)
            if w in self._state.warns:
                c = WARN if w.kind == "topic_rate" else BAD
                out.append(f"{w.kind:<14}", style=f"bold {c}").append(" ").append(w.detail)
            else:
                out.append(f"{w.kind:<14} {w.detail} · {fmt_age(age)} ago", style=DIM)
        self.query_one("#warns-list", Static).update(
            out if out.plain else Text("no warns seen", style=DIM)
        )

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
    """Glob for every per-process probe log, not the newest one: a live stack is
    one file per node and the newest file is one node of it."""
    for d in (os.environ.get("TMPDIR") or tempfile.gettempdir(), "/tmp"):
        pattern = os.path.join(d, "topic_freq.*.log")
        if glob.glob(pattern):
            return pattern
    return None


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="pulse-top",
        description="Live TUI over a ros2_pulse jsonl log (run the probe with ROS_TOPIC_STATS_FORMAT=jsonl).",
    )
    ap.add_argument(
        "file", nargs="?",
        help="probe log, or a quoted glob like '/tmp/topic_freq.*.log' "
             "(default: every $TMPDIR/topic_freq.<pid>.log)",
    )
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
