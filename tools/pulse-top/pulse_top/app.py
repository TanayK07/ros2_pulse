"""pulse-top, live TUI over the ros2_pulse probe's logs.

Pure consumer: tails the files the probe already writes (the default text format or
ROS_TOPIC_STATS_FORMAT=jsonl), parses each window, renders. No ROS dependency, no graph presence, zero cost to the
probed system beyond the file read, works over ssh and on dead logs after the fact,
which is exactly what the graph-joining monitors (ros2top, ornis, ...) cannot do.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.theme import Theme
from textual.widgets import DataTable, Footer, Static, TabbedContent, TabPane, Tree

from .model import LAG_TOL_DEFAULT, LAG_WINDOWS_DEFAULT, LogParser, StatsState, sparkline
from .reader import MultiFollower, default_log_path


@dataclass(frozen=True)
class Palette:
    """Rich styles for log-derived text, one set per theme (issue #59). The widget
    chrome follows the Textual theme through app.tcss; these colour the cells."""
    textual_theme: str
    accent: str
    good: str
    warn: str
    bad: str
    dim: str


PALETTES = {
    "dark": Palette("pulse-dark", "#b48cf2", "#7ee2a8", "#f2c96b", "#f27d72", "#7c8797"),
    # Black on white for daylight (issue #59): every colour here is dark enough to read
    # on a white background, amber included, which is why it is brown and not yellow.
    "light": Palette("pulse-light", "#6f42c1", "#116329", "#7d4e00", "#cf222e", "#57606a"),
    # The terminal's own palette: default background and foreground, ANSI colour names
    # for the accents, so whatever scheme the user picked is the scheme they get.
    "terminal": Palette("ansi-dark", "magenta", "green", "yellow", "red", "dim"),
}
THEMES = tuple(PALETTES)
THEME_ENV = "PULSE_TOP_THEME"

_PULSE_THEMES = (
    Theme(name="pulse-dark", primary="#b48cf2", foreground="#d6dde6", background="#0e1116",
          surface="#11151d", panel="#151a22", warning="#f2c96b", error="#f27d72",
          success="#7ee2a8", dark=True,
          variables={"border-blurred": "#2a3140", "text-muted": "#7c8797"}),
    Theme(name="pulse-light", primary="#6f42c1", foreground="#1f2328", background="#ffffff",
          surface="#f6f8fa", panel="#eaeef2", warning="#7d4e00", error="#cf222e",
          success="#116329", dark=False,
          variables={"border-blurred": "#d0d7de", "text-muted": "#57606a"}),
)


def resolve_theme(flag: str | None) -> str:
    """--theme wins, then $PULSE_TOP_THEME (so it can live in .bashrc), then dark."""
    name = (flag or os.environ.get(THEME_ENV) or "dark").strip().lower()
    if name not in PALETTES:
        raise ValueError(f"unknown theme {name!r}, expected one of: {', '.join(THEMES)}")
    return name


# The dark palette under its historical names, for callers that import them.
ACCENT, GOOD, WARN, BAD, DIM = (PALETTES["dark"].accent, PALETTES["dark"].good,
                                PALETTES["dark"].warn, PALETTES["dark"].bad, PALETTES["dark"].dim)

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


def warn_color(kind: str | None, pal: Palette = PALETTES["dark"]) -> str | None:
    """Colour for a warn kind: red for a measured stall or a missing node, amber for
    a rate judgement or an inference (topic_rate, recv_lag), None without a warn."""
    if kind is None:
        return None
    return pal.bad if kind in ("topic_gap", "node_missing") else pal.warn


def _short_list(paths: list[str], n: int = 4) -> str:
    names = [os.path.basename(p) for p in paths[:n]]
    more = f" (+{len(paths) - n} more)" if len(paths) > n else ""
    return ", ".join(names) + more


class PulseTopApp(App):
    TITLE = "pulse-top"
    CSS_PATH = "app.tcss"
    BINDINGS = [
        Binding("q", "quit", "quit"),
        Binding("s", "cycle_sort", "sort"),
        Binding("w", "toggle_warns_only", "warns only"),
    ]

    def __init__(self, path: str, poll_s: float = 0.5, lag_tol: float = LAG_TOL_DEFAULT,
                 lag_windows: int = LAG_WINDOWS_DEFAULT, theme: str = "dark"):
        super().__init__()
        self._pal = PALETTES[theme]
        for t in _PULSE_THEMES:
            self.register_theme(t)
        self.theme = self._pal.textual_theme
        self._path = path
        self._poll_s = poll_s
        self._follower = MultiFollower(path)
        # One parser per file: a text window spans lines and a poll can end mid-block.
        self._parsers: dict[str, LogParser] = {}
        self._status: tuple[str, str] | None = None
        self._state = StatsState(lag_tol=lag_tol, lag_windows=lag_windows)
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
        yield Static(id="notice")
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
        self._refresh_status()
        self.set_interval(self._poll_s, self._tick)

    def _tick(self) -> None:
        # A poll reads file by file, so a backlog (attach, post-mortem) would
        # hand the model the publisher's whole file before the subscriber's
        # first window. Order the batch by ts_ns: the recv_lag pairing is a
        # question of time, and the source file is the tracker's provenance.
        batch = []
        for path, line in self._follower.poll_tagged():
            parser = self._parsers.get(path)
            if parser is None:
                parser = self._parsers[path] = LogParser()
            for w in parser.feed(line):
                batch.append((w.ts_ns, path, w))
        for _, path, w in sorted(batch, key=lambda b: b[0]):
            self._state.apply(w, source=path)
        if batch:
            self._refresh_all()
        else:
            # Every tick, not only after a window (issue #58): files found, files
            # that are not probe output, waiting for a first window, all must show.
            self._refresh_status()

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
        self._refresh_status()
        self._refresh_table()
        self._refresh_warns_strip()
        self._refresh_sidebar()
        self._refresh_tree()
        self._refresh_nodes()
        self._refresh_warns_tab()

    def _notice_text(self) -> str:
        """Why the table is empty, or which files are not probe output. Empty when
        there is nothing to say. Never a silent "0 files" (issue #58)."""
        files = self._follower.files
        foreign = [p for p in files
                   if (pr := self._parsers.get(p)) is not None and pr.unrecognised and not pr.windows]
        if foreign:
            return (f"{len(foreign)} of {len(files)} file(s) are not ros2_pulse probe output, "
                    f"skipped: {_short_list(foreign)}. Expected probe windows "
                    "('# ts_ns=' text blocks or jsonl records).")
        if self._state.windows_seen:
            return ""
        if not files:
            return (f"no files match {self._path} yet. Is the probe preloaded "
                    "(LD_PRELOAD=libros2_pulse.so) and writing there? It writes "
                    "$TMPDIR/topic_freq.<pid>.log, or /tmp when TMPDIR is unset.")
        return (f"{len(files)} file(s) found, waiting for the first window: the probe writes "
                "one per ROS_TOPIC_STATISTICS_PUBLISH_PERIOD (5 s by default).")

    def _refresh_status(self) -> None:
        s, pal = self._state, self._pal
        n = len(s.warns)
        # Counts before the path: a long glob is what gets cut, never the file count.
        # Text, not markup: the path is user input and a glob may hold '['.
        top = Text.assemble(
            ("pulse-top ", f"bold {pal.accent}"),
            (f"{len(self._follower.files)} file(s) · window {s.window_s:.1f}s · "
             f"{s.windows_seen} windows  ", pal.dim),
            (f"{n} warns", f"bold {pal.warn}") if n else ("0 warns", pal.dim),
            (f"  {self._path}", pal.dim),
        )
        note = self._notice_text()
        if (top.plain, note) == self._status:
            return  # nothing changed: no repaint (twice a second over ssh adds up)
        self._status = (top.plain, note)
        self.query_one("#topbar", Static).update(top)
        notice = self.query_one("#notice", Static)
        notice.display = bool(note)
        notice.update(Text(note, style=f"bold {pal.warn}"))

    def _row_cells(self, name: str, st) -> tuple:
        t = st.latest
        # The probe omits silent topics per window: absence is "no traffic seen",
        # not "still at the old rate". Render stale rows as stale, never repeat
        # the last measurement as if current (PR #32 review). Age is wall time
        # from the window timestamps, windows from every process interleave.
        if self._state.is_stale(name):
            stale = Text("—", justify="right", style=self._pal.dim)
            return (
                Text(f"{name}  · stale {fmt_age(self._state.age_s(name))}", style=self._pal.dim),
                stale, stale.copy(), stale.copy(), stale.copy(),
                Text(sparkline(st.rate_history), style=self._pal.dim),
            )
        color = warn_color(st.warn_kind, self._pal)
        gap = t.pub_max_dt_ms if t.pub_max_dt_ms is not None else t.recv_max_dt_ms
        # recv_lag: the eye should land on the column that is low.
        recv_style = f"bold {self._pal.warn}" if st.warn_kind == "recv_lag" else ""
        return (
            Text(name, style=f"bold {color}" if color else ""),
            Text(fmt(t.pub_inter_hz), justify="right"),
            Text(fmt(t.pub_intra_hz), justify="right",
                 style=self._pal.good if t.pub_intra_hz else self._pal.dim),
            Text(fmt(t.recv_inter_hz), justify="right", style=recv_style),
            Text(fmt(gap), justify="right",
                 style=f"bold {self._pal.bad}" if st.warn_kind == "topic_gap" else self._pal.dim),
            Text(sparkline(st.rate_history), style=color or self._pal.dim),
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
        out.append("WARNS · structured from the probe", style=f"{self._pal.dim} bold")
        for w in self._state.warns:
            out.append("\n").append(w.kind, style=f"bold {warn_color(w.kind, self._pal)}").append(" ").append(w.detail)
        if not self._state.warns:
            out.append("\n").append("none", style=self._pal.dim)
        self.query_one("#warns-strip", Static).update(out)

    def _refresh_sidebar(self) -> None:
        sb = self.query_one("#sidebar", Static)
        st = self._state.topics.get(self._selected or "")
        if st is None:
            sb.update(f"[{self._pal.dim}]SELECTED\n(no topic)[/]")
            return
        t = st.latest
        spark = sparkline(st.rate_history, width=24)
        color = self._pal.bad if st.warn_kind else self._pal.accent
        # Every lagging subscriber process of this topic, worst first: the row
        # shows the worst and how many there are, the source row names the worst.
        lags = sorted((w for w in self._state.warns if w.kind == "recv_lag" and w.topic == self._selected),
                      key=lambda w: -(w.deficit or 0.0))
        lag = lags[0] if lags else None
        rows = [
            ("pub inter ", f"{fmt(t.pub_inter_hz)} Hz"),
            ("pub intra ", f"{fmt(t.pub_intra_hz)} Hz"),
            ("recv inter", f"{fmt(t.recv_inter_hz)} Hz"),
            ("recv intra", f"{fmt(t.recv_intra_hz)} Hz"),
            ("pub gap   ", f"{fmt(t.pub_max_dt_ms)} ms"),
            ("recv gap  ", f"{fmt(t.recv_max_dt_ms)} ms"),
            ("endpoint  ", "seen" if t.recv_endpoint_seen else "—"),
            ("recv lag  ", self._lag_cell(lags)),
        ]
        if lag and lag.source:  # which subscriber process: the log file is the only provenance
            rows.append(("lag source", os.path.basename(lag.source)))
        out = Text()
        out.append("SELECTED", style=f"{self._pal.dim} bold").append("\n")
        out.append(self._selected or "", style=f"bold {color}")
        out.append("\n\n").append(spark, style=color).append("\n")
        for label, value in rows:
            out.append("\n").append(label, style=self._pal.dim).append(" ").append(value)
        sb.update(out)

    @staticmethod
    def _lag_cell(lags: list) -> str:
        if not lags:
            return "—"
        more = f" ({len(lags)} sources)" if len(lags) > 1 else ""
        return f"-{round(lags[0].deficit * 100)}% over {lags[0].windows}w{more}"

    def _leaf_label(self, part: str, st) -> Text:
        # Text.assemble, not markup: namespace parts are log-derived (PR #32 review).
        color = warn_color(st.warn_kind, self._pal) or self._pal.dim
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
            out.append("■ " if alive else "□ ", style=self._pal.good if alive else self._pal.bad)
            out.append(name)
            if not alive:
                out.append(" missing", style=self._pal.bad)
        self.query_one("#nodes-list", Static).update(
            out if out.plain else Text("no NODE lines yet", style=self._pal.dim)
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
            # Identity, not equality: a cleared recv_lag episode keeps its last
            # numbers, and a later episode with the same numbers is a different
            # object that must not render its predecessor as live.
            if any(w is live for live in self._state.warns):
                out.append(f"{w.kind:<14}", style=f"bold {warn_color(w.kind, self._pal)}")
                out.append(" ").append(w.detail)
            else:
                out.append(f"{w.kind:<14} {w.detail} · {fmt_age(age)} ago", style=self._pal.dim)
        self.query_one("#warns-list", Static).update(
            out if out.plain else Text("no warns seen", style=self._pal.dim)
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


def main() -> int:
    ap = argparse.ArgumentParser(
        prog="pulse-top",
        description="Live TUI over ros2_pulse probe logs, text or jsonl.",
    )
    ap.add_argument(
        "file", nargs="?",
        help="probe log, or a quoted glob like '/tmp/topic_freq.*.log' "
             "(default: every $TMPDIR/topic_freq.<pid>.log)",
    )
    ap.add_argument("--demo", action="store_true", help="run against a self-generated demo log")
    ap.add_argument("--poll", type=float, default=0.5, help="file poll interval seconds (default 0.5)")
    ap.add_argument(
        "--lag-tol", type=float, default=LAG_TOL_DEFAULT, metavar="FRACTION",
        help=f"recv_lag: relative deficit of callbacks under the publish rate that counts a window "
             f"as lagging (default {LAG_TOL_DEFAULT})",
    )
    ap.add_argument(
        "--lag-windows", type=int, default=LAG_WINDOWS_DEFAULT, metavar="N",
        help=f"recv_lag: consecutive lagging windows before the warn fires, 0 disables "
             f"(default {LAG_WINDOWS_DEFAULT})",
    )
    ap.add_argument(
        "--theme", choices=THEMES, default=None,
        help=f"colours: dark (default), light (black on white, for daylight), or terminal "
             f"(your terminal's own background and palette). Also read from ${THEME_ENV}.",
    )
    args = ap.parse_args()
    try:
        theme = resolve_theme(args.theme)
    except ValueError as e:
        ap.error(f"{THEME_ENV}: {e}")

    if args.demo:
        from .demo import start_demo_writer

        path = start_demo_writer()
    else:
        path = args.file or default_log_path()
        if path is None:
            print(
                "pulse-top: no probe log ($TMPDIR/topic_freq.<pid>.log) found. Start the "
                "probe, pass a path or a quoted glob, or try --demo.",
                file=sys.stderr,
            )
            return 2

    PulseTopApp(path, poll_s=args.poll, lag_tol=args.lag_tol, lag_windows=args.lag_windows,
                theme=theme).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
