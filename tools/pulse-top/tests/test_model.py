"""Model-layer tests: jsonl parsing and rolling state.

The jsonl schema these tests pin is the probe's (CHANGELOG 0.3.0, README "JSON Lines
output"): ts_ns as a decimal STRING, absent key = "not measured" (never 0), topics/
nodes/warns always present, structured warns. A TUI that misread absence as zero
would invent data the probe deliberately refused to claim.
"""

import pytest

from pulse_top.model import StatsState, parse_jsonl_line, sparkline

RECORD = (
    '{"ts_ns":"1782887153899445923","window_s":5.000,'
    '"topics":[{"topic":"/scan","pub_inter_hz":19.800000,"pub_intra_hz":0.000000,'
    '"recv_inter_hz":19.800000,"recv_intra_hz":0.000000,"recv_endpoint_seen":true,'
    '"pub_max_dt_ms":812.400},'
    '{"topic":"/points","pub_inter_hz":10.000000,"pub_intra_hz":10.000000}],'
    '"nodes":["/perception","/planner"],'
    '"warns":[{"kind":"topic_gap","topic":"/scan","max_dt_ms":812.400,"max_gap_ms":250},'
    '{"kind":"node_missing","node":"/localization"}]}'
)


class TestParse:
    def test_parses_full_record(self):
        w = parse_jsonl_line(RECORD)
        assert w is not None
        assert w.ts_ns == 1782887153899445923  # string-encoded int64, no digit loss
        assert w.window_s == 5.0
        assert [t.topic for t in w.topics] == ["/scan", "/points"]
        assert w.nodes == ["/perception", "/planner"]
        assert len(w.warns) == 2

    def test_absent_means_unmeasured_not_zero(self):
        w = parse_jsonl_line(RECORD)
        scan = w.topics[0]
        assert scan.pub_max_dt_ms == 812.4
        points = w.topics[1]
        assert points.pub_max_dt_ms is None      # absent key stays None
        assert points.recv_inter_hz is None      # never coerced to 0.0
        assert points.recv_endpoint_seen is False

    def test_structured_warns(self):
        w = parse_jsonl_line(RECORD)
        gap = w.warns[0]
        assert gap.kind == "topic_gap"
        assert gap.topic == "/scan"
        assert gap.detail  # human-renderable, non-empty

    @pytest.mark.parametrize(
        "line",
        [
            "",
            "not json",
            "[1,2,3]",
            '{"ts_ns":"42"}',                  # truncated: no window_s
            '{"ts_ns":"x","window_s":1.0,"topics":[],"nodes":[],"warns":[]}',
            "# ts_ns=100 window_s=5.000",      # text-format line: not ours
        ],
    )
    def test_garbage_returns_none_never_raises(self, line):
        assert parse_jsonl_line(line) is None


class TestState:
    def test_apply_accumulates_history(self):
        s = StatsState(history=8)
        for _ in range(3):
            s.apply(parse_jsonl_line(RECORD))
        assert s.windows_seen == 3
        assert list(s.topics["/scan"].rate_history) == [19.8, 19.8, 19.8]

    def test_warn_topics_flagged(self):
        s = StatsState()
        s.apply(parse_jsonl_line(RECORD))
        assert s.topics["/scan"].warn_kind == "topic_gap"
        assert s.topics["/points"].warn_kind is None

    def test_missing_node_from_warns(self):
        s = StatsState()
        s.apply(parse_jsonl_line(RECORD))
        assert s.nodes["/perception"] is True
        assert s.nodes["/localization"] is False

    def test_stale_warn_clears_next_window(self):
        s = StatsState()
        s.apply(parse_jsonl_line(RECORD))
        clean = parse_jsonl_line(
            '{"ts_ns":"7","window_s":5.0,'
            '"topics":[{"topic":"/scan","pub_inter_hz":20.0,"pub_intra_hz":0.0}],'
            '"nodes":["/perception"],"warns":[]}'
        )
        s.apply(clean)
        assert s.topics["/scan"].warn_kind is None
        assert s.warns == []


class TestSparkline:
    def test_renders_full_range(self):
        line = sparkline([0.0, 5.0, 10.0], width=3)
        assert line[0] == "▁" and line[-1] == "█"

    def test_flat_series_is_mid_block_not_crash(self):
        assert sparkline([5.0, 5.0], width=2) == "▄▄"

    def test_empty_pads_to_width(self):
        assert sparkline([], width=4) == "    "
