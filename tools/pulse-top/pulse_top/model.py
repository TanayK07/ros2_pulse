"""Parse the probe's jsonl windows and keep rolling per-topic state.

Schema contract (probe CHANGELOG 0.3.0, README "JSON Lines output"): ts_ns is a
decimal string (int64-safe), an ABSENT per-topic key means "not measured", never
zero, and topics/nodes/warns are always present. This module preserves that
absence-vs-zero distinction with Optionals; rendering absence as 0.0 would claim
a measurement the probe deliberately withheld.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass, field

BLOCKS = "▁▂▃▄▅▆▇█"

# recv_lag (issue #50): relative deficit above which a recv window counts as
# lagging, and the consecutive lagging windows before the warn fires. Three
# absorbs one bad window with margin: a subscriber that starts after the
# publisher has one partial first window, one that exits has one partial last.
LAG_TOL_DEFAULT = 0.10
LAG_WINDOWS_DEFAULT = 3
# The clause every recv_lag detail ends with. The probe counts callbacks, not
# wire samples, so it cannot name lost messages or tell a QoS drop from an
# executor that is behind; the text must never claim more than that.
LAG_LIMIT_TEXT = "callbacks see fewer than published; drop or backlog, probe cannot tell which"


@dataclass
class TopicWindow:
    topic: str
    pub_inter_hz: float | None = None
    pub_intra_hz: float | None = None
    recv_inter_hz: float | None = None
    recv_intra_hz: float | None = None
    recv_endpoint_seen: bool = False
    pub_max_dt_ms: float | None = None
    recv_max_dt_ms: float | None = None


@dataclass
class Warn:
    kind: str
    topic: str | None = None
    node: str | None = None
    detail: str = ""
    # recv_lag only (issue #50): derived by this consumer, never written by the
    # probe. Defaulted so the three probe kinds construct unchanged.
    pub_hz: float | None = None
    recv_hz: float | None = None
    deficit: float | None = None
    windows: int = 0
    source: str | None = None


@dataclass
class Window:
    ts_ns: int
    window_s: float
    topics: list[TopicWindow]
    nodes: list[str]
    warns: list[Warn]


def _warn_detail(w: dict) -> str:
    kind = w.get("kind", "?")
    if kind == "topic_rate":
        lo, hi, hz = w.get("min_hz"), w.get("max_hz"), w.get("hz")
        if lo is not None and hz is not None and hz < lo:
            bound = f"< min {lo}Hz"
        elif hi is not None and hz is not None and hz > hi:
            bound = f"> max {hi}Hz"
        else:  # the probe never warns in-range; say what is known, claim nothing
            bound = f"bounds [{lo}, {hi}]Hz"
        shown = f"{hz:.1f}" if isinstance(hz, (int, float)) else hz
        return f"{w.get('topic')} {shown}Hz {bound}"
    if kind == "topic_gap":
        return f"{w.get('topic')} max_dt {w.get('max_dt_ms')}ms > max_gap {w.get('max_gap_ms')}ms"
    if kind == "node_missing":
        return f"{w.get('node')} expected alive, not seen"
    return json.dumps(w)


def _lag_detail(topic: str, pub_hz: float, recv_hz: float, deficit: float, windows: int) -> str:
    unit = "window" if windows == 1 else "windows"
    return (f"{topic} recv {recv_hz:.1f}Hz < pub {pub_hz:.1f}Hz "
            f"(-{round(deficit * 100)}%, {windows} {unit}) · {LAG_LIMIT_TEXT}")


def parse_jsonl_line(line: str) -> Window | None:
    """One jsonl record -> Window; anything malformed -> None, never an exception.

    Mirrors the C++ log_reader's tolerance: a truncated tail after a crash or a
    foreign line must not kill a live dashboard.
    """
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        rec = json.loads(line)
        if not isinstance(rec, dict):
            return None
        ts_ns = int(rec["ts_ns"])
        window_s = float(rec["window_s"])
        topics = [
            TopicWindow(
                topic=t["topic"],
                pub_inter_hz=t.get("pub_inter_hz"),
                pub_intra_hz=t.get("pub_intra_hz"),
                recv_inter_hz=t.get("recv_inter_hz"),
                recv_intra_hz=t.get("recv_intra_hz"),
                recv_endpoint_seen=bool(t.get("recv_endpoint_seen", False)),
                pub_max_dt_ms=t.get("pub_max_dt_ms"),
                recv_max_dt_ms=t.get("recv_max_dt_ms"),
            )
            for t in rec.get("topics", [])
        ]
        warns = [
            Warn(
                kind=w.get("kind", "?"),
                topic=w.get("topic"),
                node=w.get("node"),
                detail=_warn_detail(w),
            )
            for w in rec.get("warns", [])
        ]
        return Window(ts_ns, window_s, topics, list(rec.get("nodes", [])), warns)
    except (KeyError, ValueError, TypeError):
        return None


@dataclass
class TopicState:
    latest: TopicWindow
    rate_history: deque = field(default_factory=lambda: deque(maxlen=60))
    warn_kind: str | None = None
    # Timestamp of the newest window this topic appeared in, and that window's
    # period. The probe omits silent topics per window, so "absent" is a
    # statement, the view renders a topic older than ~1.5 windows as STALE
    # instead of repeating the old rate as if current (PR #32 review). Age is
    # measured in TIME, never in windows counted: windows from every probed
    # process interleave in a shared log, so "3 windows ago" says nothing
    # (77-node Orin stack, 2026-08-23).
    last_ts_ns: int = 0
    window_s: float = 0.0
    # Timestamp of the last rate_history sample. One sample per window PERIOD,
    # whichever process's window arrives first in it, twenty subscribers
    # flushing /tf_static in the same period are one data point, not twenty.
    hist_ts_ns: int = 0
    # Timestamp and period of the newest window that carried the PUB side. The
    # merged record cannot say whether its pub fields are from the last window
    # or from ten minutes ago, and a dead publisher's last rate paired with a
    # live recv 0.0 is exactly the recv_lag false positive to gate out.
    pub_ts_ns: int = 0
    pub_window_s: float = 0.0

    @property
    def rate(self) -> float:
        """Headline rate: publish-side if measured, else receive-side, else 0."""
        for v in (self.latest.pub_inter_hz, self.latest.recv_inter_hz):
            if v is not None:
                return v
        return 0.0


_PUB_FIELDS = ("pub_inter_hz", "pub_intra_hz", "pub_max_dt_ms")
_RECV_FIELDS = ("recv_inter_hz", "recv_intra_hz", "recv_max_dt_ms", "recv_endpoint_seen")


def _merge_sides(into: TopicWindow, tw: TopicWindow) -> None:
    """Fold a window's topic record into the retained one, side by side.

    One process publishes /tf_static; twenty receive it. Each process's window
    carries only its own side, so a recv-only window must refresh the recv
    fields and leave the pub fields, learned from the publisher's window,
    untouched. Absence of a whole side is "this process had no such endpoint",
    not "the rate is now unknown".
    """
    if into is tw:
        return
    if _has_pub(tw):
        for f in _PUB_FIELDS:
            setattr(into, f, getattr(tw, f))
    if _has_recv(tw):
        for f in _RECV_FIELDS:
            setattr(into, f, getattr(tw, f))


def _has_pub(tw: TopicWindow) -> bool:
    return any(getattr(tw, f) is not None for f in _PUB_FIELDS)


def _has_recv(tw: TopicWindow) -> bool:
    return tw.recv_endpoint_seen or any(getattr(tw, f) is not None for f in _RECV_FIELDS[:3])


def _has_recv_rate(tw: TopicWindow) -> bool:
    """A receive side WITH a rate in it. recv_endpoint_seen or a recv gap alone is a
    side with no measurement, and absent is never a 100% deficit."""
    return tw.recv_inter_hz is not None or tw.recv_intra_hz is not None


@dataclass
class _LagTrack:
    """recv_lag state for one (topic, source log) pair: the consecutive-window streak
    and, while active, the live Warn. The same object sits in recent_warns so the
    Warns tab treats it as live while its numbers update, and it freezes at its
    last numbers when it clears."""
    streak: int = 0
    warn: Warn | None = None
    # Newest recv window evaluated, and its period, for the time-based expiry.
    ts_ns: int = 0
    window_s: float = 0.0


class StatsState:
    """Rolling view over the window stream: per-topic history, node liveness, warns."""

    def __init__(self, history: int = 60, lag_tol: float = LAG_TOL_DEFAULT,
                 lag_windows: int = LAG_WINDOWS_DEFAULT):
        self._history = history
        self.lag_tol = lag_tol
        self.lag_windows = lag_windows
        self.topics: dict[str, TopicState] = {}
        self.nodes: dict[str, bool] = {}
        self.warns: list[Warn] = []
        # (ts_ns_when_fired, warn), bounded retention so a one-window
        # transient (a single stall) stays readable with an age instead of
        # blinking for one window period (PR #32 review).
        self.recent_warns: deque = deque(maxlen=50)
        # Keyed by (topic, source log path): one file per process by default, so
        # a healthy subscriber process and a lagging one do not alternate and
        # reset each other's streak. A shared file has no provenance (None).
        self._lag: dict[tuple[str, str | None], _LagTrack] = {}
        self.windows_seen = 0
        self.window_s = 0.0
        # Newest timestamp seen across all processes' windows. Monotone: a late
        # flush from a slow process must not rewind everyone else's age.
        self.last_ts_ns = 0

    # A topic is stale once more than this many of its own window periods have
    # passed since its last window: one missed flush plus scheduling slack.
    STALE_FACTOR = 1.5

    def age_s(self, topic: str) -> float:
        """Seconds between the newest window seen and this topic's last window."""
        return max(0, self.last_ts_ns - self.topics[topic].last_ts_ns) / 1e9

    def is_stale(self, topic: str) -> bool:
        st = self.topics[topic]
        return not self._fresh(max(0, self.last_ts_ns - st.last_ts_ns), st.window_s)

    def _fresh(self, age_ns: float, window_s: float) -> bool:
        """Within STALE_FACTOR periods of the given window. The one rule behind
        stale rows, recv_lag pub/recv pairing and recv_lag expiry."""
        return age_ns <= self.STALE_FACTOR * window_s * 1e9

    def warn_age_s(self, fired_ts_ns: int) -> float:
        return max(0, self.last_ts_ns - fired_ts_ns) / 1e9

    def apply(self, window: Window | None, source: str | None = None) -> None:
        """Fold one window in. source is the log file it came from (None for a
        shared file); it is the provenance the recv_lag trackers are keyed by."""
        if window is None:
            return
        self.windows_seen += 1
        self.window_s = window.window_s
        self.last_ts_ns = max(self.last_ts_ns, window.ts_ns)
        for w in window.warns:
            self.recent_warns.append((window.ts_ns, w))

        for tw in window.topics:
            st = self.topics.get(tw.topic)
            if st is None:
                st = TopicState(latest=tw)
                st.rate_history = deque(maxlen=self._history)
                self.topics[tw.topic] = st
            _merge_sides(st.latest, tw)
            if _has_pub(tw) and window.ts_ns >= st.pub_ts_ns:
                st.pub_ts_ns = window.ts_ns
                st.pub_window_s = window.window_s
            if window.ts_ns >= st.last_ts_ns:
                st.last_ts_ns = window.ts_ns
                st.window_s = window.window_s
            if window.ts_ns - st.hist_ts_ns >= 0.5 * window.window_s * 1e9:
                st.hist_ts_ns = window.ts_ns
                st.rate_history.append(st.rate)

        self.warns = window.warns + self._eval_recv_lag(window, source)
        # Warns are a question about NAMES, answer it with a name map, never
        # record equality (PR #32 review: dataclass float-equality here silently
        # changes behavior on the first TopicWindow schema change). A topic absent
        # from this window but warned about (a gap on a stalled topic that emitted
        # nothing, a recv_lag held by its tracker) still carries its warn; the
        # probe's own verdict outranks a derived one.
        warned = {w.topic: w.kind for w in reversed(self.warns) if w.topic}
        for name, st in self.topics.items():
            st.warn_kind = warned.get(name)

        for n in window.nodes:
            self.nodes[n] = True
        for w in window.warns:
            if w.kind == "node_missing" and w.node:
                self.nodes[w.node] = False

    def _eval_recv_lag(self, window: Window, source: str | None) -> list[Warn]:
        """Derive recv_lag warns (issue #50) from this window's recv sides.

        P is the newest pub observation for the topic, R this window's recv rate,
        each combined by the probe's own rule (pub: busier path, recv: sum). Only
        a recv window advances or resets its tracker; windows from other processes
        neither. A pub observation older than STALE_FACTOR periods (dead or idle
        publisher, KNOWN_ISSUES #12: recv reads an explicit 0.0) cannot pair.
        """
        if self.lag_windows <= 0:
            return []
        if window.window_s > 0:  # the parser tolerates a malformed 0; the tick must survive it
            for tw in window.topics:
                if _has_recv_rate(tw):
                    self._track_recv(tw, source, window)
        self._expire_lag()
        return [t.warn for t in self._lag.values() if t.warn is not None]

    def _track_recv(self, tw: TopicWindow, source: str | None, window: Window) -> None:
        key = (tw.topic, source)
        track = self._lag.get(key)
        if track is None:
            track = self._lag[key] = _LagTrack()
        track.ts_ns, track.window_s = window.ts_ns, window.window_s
        recv = (tw.recv_inter_hz or 0.0) + (tw.recv_intra_hz or 0.0)
        pub = self._paired_pub_hz(tw.topic, window)
        self._step_lag(track, tw.topic, source, pub, recv, window)

    def _paired_pub_hz(self, topic: str, window: Window) -> float | None:
        """P for a recv window, or None when the topic has no pub observation that
        can pair with it. Window boundaries are per process; two equal periods at
        any phase offset overlap by at least period - |offset|, so within
        STALE_FACTOR of the period the two means agree at steady state. Periods
        more than STALE_FACTOR apart never pair: a publisher flushing every 5 s
        against 1 s subscriber windows reads 100, 0, 0, 0, 0 on a healthy pipe."""
        st = self.topics[topic]
        if st.pub_ts_ns == 0:
            return None
        long_s = max(window.window_s, st.pub_window_s)
        short_s = min(window.window_s, st.pub_window_s)
        if short_s <= 0 or long_s > self.STALE_FACTOR * short_s:
            return None
        if not self._fresh(abs(window.ts_ns - st.pub_ts_ns), long_s):
            return None
        return max(st.latest.pub_inter_hz or 0.0, st.latest.pub_intra_hz or 0.0)

    def _step_lag(self, track: _LagTrack, topic: str, source: str | None,
                  pub: float | None, recv: float, window: Window) -> None:
        if pub is None or pub <= 0:
            track.streak, track.warn = 0, None
            return
        deficit = (pub - recv) / pub
        # Two thresholds: the relative tolerance, and an absolute floor of two
        # messages per window, since one message can cross a window boundary
        # from phase offset alone (a 20% swing at 1 Hz on 5 s windows). A
        # shortfall inside the floor is healthy whatever the ratio says.
        inside_floor = pub - recv <= 2.0 / window.window_s
        if inside_floor or deficit <= self.lag_tol:
            if inside_floor or deficit <= self.lag_tol / 2:
                track.streak, track.warn = 0, None
            return  # else hysteresis band: hold, neither flap nor re-fire
        track.streak += 1
        if track.streak < self.lag_windows:
            return
        if track.warn is None:  # transition to active: one recent_warns entry, not one per window
            track.warn = Warn("recv_lag", topic=topic, source=source)
            self.recent_warns.append((window.ts_ns, track.warn))
        w = track.warn
        w.pub_hz, w.recv_hz, w.deficit, w.windows = pub, recv, deficit, track.streak
        w.detail = _lag_detail(topic, pub, recv, deficit, track.streak)

    # A tracker idle for this many of its own periods is dropped, or one per
    # process that ever flushed would be kept and scanned on every window.
    LAG_PRUNE_PERIODS = 10

    def _expire_lag(self) -> None:
        # A subscriber process that exits stops flushing while the publisher's
        # windows keep the row live; a warn with no recv observation behind it
        # for more than STALE_FACTOR periods is cleared, by time, like stale rows.
        for key, track in list(self._lag.items()):
            age_ns = self.last_ts_ns - track.ts_ns
            if age_ns > self.LAG_PRUNE_PERIODS * track.window_s * 1e9:
                del self._lag[key]
            elif track.warn is not None and not self._fresh(age_ns, track.window_s):
                track.streak, track.warn = 0, None


def sparkline(values, width: int = 16) -> str:
    """Render a series as block characters, right-aligned, padded to width."""
    vals = list(values)[-width:]
    if not vals:
        return " " * width
    lo, hi = min(vals), max(vals)
    span = hi - lo
    if span <= 0:
        line = BLOCKS[3] * len(vals)
    else:
        line = "".join(BLOCKS[round((v - lo) / span * (len(BLOCKS) - 1))] for v in vals)
    return line.rjust(width)
