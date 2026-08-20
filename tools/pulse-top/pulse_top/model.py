"""Parse the probe's jsonl windows and keep rolling per-topic state.

Schema contract (probe CHANGELOG 0.3.0, README "JSON Lines output"): ts_ns is a
decimal string (int64-safe), an ABSENT per-topic key means "not measured" — never
zero — and topics/nodes/warns are always present. This module preserves that
absence-vs-zero distinction with Optionals; rendering absence as 0.0 would claim
a measurement the probe deliberately withheld.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass, field

BLOCKS = "▁▂▃▄▅▆▇█"


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
        lo, hi = w.get("min_hz"), w.get("max_hz")
        bound = f"< min {lo}Hz" if lo is not None and w.get("hz", 0) < lo else f"> max {hi}Hz"
        return f"{w.get('topic')} {w.get('hz')}Hz {bound}"
    if kind == "topic_gap":
        return f"{w.get('topic')} max_dt {w.get('max_dt_ms')}ms > max_gap {w.get('max_gap_ms')}ms"
    if kind == "node_missing":
        return f"{w.get('node')} expected alive, not seen"
    return json.dumps(w)


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

    @property
    def rate(self) -> float:
        """Headline rate: publish-side if measured, else receive-side, else 0."""
        for v in (self.latest.pub_inter_hz, self.latest.recv_inter_hz):
            if v is not None:
                return v
        return 0.0


class StatsState:
    """Rolling view over the window stream: per-topic history, node liveness, warns."""

    def __init__(self, history: int = 60):
        self._history = history
        self.topics: dict[str, TopicState] = {}
        self.nodes: dict[str, bool] = {}
        self.warns: list[Warn] = []
        self.windows_seen = 0
        self.window_s = 0.0
        self.last_ts_ns = 0

    def apply(self, window: Window | None) -> None:
        if window is None:
            return
        self.windows_seen += 1
        self.window_s = window.window_s
        self.last_ts_ns = window.ts_ns
        self.warns = window.warns

        warned = {w.topic: w.kind for w in window.warns if w.topic}
        for tw in window.topics:
            st = self.topics.get(tw.topic)
            if st is None:
                st = TopicState(latest=tw)
                st.rate_history = deque(maxlen=self._history)
                self.topics[tw.topic] = st
            st.latest = tw
            st.warn_kind = warned.get(tw.topic)
            st.rate_history.append(st.rate)
        # A topic absent from this window but warned about (e.g. gap on a stalled
        # topic that emitted nothing) still carries its warn.
        for topic, kind in warned.items():
            if topic in self.topics and self.topics[topic].latest not in window.topics:
                self.topics[topic].warn_kind = kind
        for name, st in self.topics.items():
            if name not in {t.topic for t in window.topics} and name not in warned:
                st.warn_kind = None

        for n in window.nodes:
            self.nodes[n] = True
        for w in window.warns:
            if w.kind == "node_missing" and w.node:
                self.nodes[w.node] = False


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
