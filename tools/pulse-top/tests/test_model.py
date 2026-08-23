"""Model-layer tests: jsonl parsing and rolling state.

The jsonl schema these tests pin is the probe's (CHANGELOG 0.3.0, README "JSON Lines
output"): ts_ns as a decimal STRING, absent key = "not measured" (never 0), topics/
nodes/warns always present, structured warns. A TUI that misread absence as zero
would invent data the probe deliberately refused to claim.
"""

import json

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


class TestWarnDetail:
    def detail(self, hz, lo=19.0, hi=22.0):
        rec = {"ts_ns": "1", "window_s": 1.0, "topics": [], "nodes": [],
               "warns": [{"kind": "topic_rate", "topic": "/cmd_vel", "hz": hz, "min_hz": lo, "max_hz": hi}]}
        return parse_jsonl_line(json.dumps(rec)).warns[0].detail

    def test_below_min_names_the_min(self):
        assert self.detail(10.0).endswith("< min 19.0Hz")

    def test_above_max_names_the_max(self):
        assert self.detail(25.0).endswith("> max 22.0Hz")

    def test_rate_is_rounded_for_humans(self):
        # The probe emits 6 decimals; a warn line is read by a person at 3 a.m.
        assert self.detail(17.345761).startswith("/cmd_vel 17.3Hz ")

    def test_in_range_never_claims_a_bound_was_crossed(self):
        # A rate inside [min, max] cannot be "> max"; say what is known instead.
        d = self.detail(20.0)
        assert "> max" not in d and "< min" not in d
        assert "19.0" in d and "22.0" in d


class TestState:
    def test_apply_accumulates_history(self):
        s = StatsState(history=8)
        for i in range(3):  # one window per period; same-period repeats are one sample
            s.apply(parse_jsonl_line(RECORD.replace('"ts_ns":"1782887153899445923"',
                                                    f'"ts_ns":"{1782887153899445923 + i * 5_000_000_000}"')))
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


class TestStaleness:
    # Windows from different processes interleave in a shared log (or in merged
    # per-pid logs): 77 nodes x one window each per period. Staleness must be
    # measured in TIME from the window timestamps, never in windows counted,
    # a 5 Hz topic showed "stale 3w" on a 77-node Orin stack (2026-08-23) because
    # three other processes' windows had landed since its own.
    def w(self, ts_s, topics):
        tt = ",".join(f'{{"topic":"{n}","pub_inter_hz":{hz}}}' for n, hz in topics)
        return parse_jsonl_line(
            f'{{"ts_ns":"{int(ts_s * 1e9)}","window_s":5.0,"topics":[{tt}],"nodes":[],"warns":[]}}'
        )

    def test_interleaved_windows_do_not_age_a_topic(self):
        s = StatsState()
        s.apply(self.w(100.0, [("/a", 5.0)]))
        s.apply(self.w(100.1, [("/b", 1.0)]))   # another process, same period
        s.apply(self.w(100.2, [("/c", 1.0)]))
        assert s.age_s("/a") == pytest.approx(0.2)
        assert not s.is_stale("/a")

    def test_topic_is_stale_once_more_than_a_window_and_a_half_passes(self):
        s = StatsState()
        s.apply(self.w(100.0, [("/a", 5.0)]))
        s.apply(self.w(107.0, [("/b", 1.0)]))   # 7.0 s < 1.5 x 5 s
        assert not s.is_stale("/a")
        s.apply(self.w(108.0, [("/b", 1.0)]))   # 8.0 s > 7.5 s
        assert s.is_stale("/a")
        assert s.age_s("/a") == pytest.approx(8.0)

    def test_out_of_order_window_does_not_rewind_the_clock(self):
        s = StatsState()
        s.apply(self.w(100.0, [("/a", 5.0)]))
        s.apply(self.w(110.0, [("/b", 1.0)]))
        s.apply(self.w(101.0, [("/c", 1.0)]))   # late flush from a slow process
        assert s.age_s("/a") == pytest.approx(10.0)


class TestRateHistorySampling:
    # rate_history feeds the sparkline: one sample per topic per window PERIOD.
    # Twenty subscribers each flushing a window for /tf_static in the same
    # period must not push twenty samples, that fills the 60-slot history in
    # three periods and changes the sparkline on every interleaved window.
    def w(self, ts_s, hz):
        return parse_jsonl_line(
            f'{{"ts_ns":"{int(ts_s * 1e9)}","window_s":5.0,'
            f'"topics":[{{"topic":"/a","recv_inter_hz":{hz}}}],"nodes":[],"warns":[]}}'
        )

    def test_same_period_windows_add_one_sample(self):
        s = StatsState()
        for ts in (100.0, 100.1, 100.2):
            s.apply(self.w(ts, 2.0))
        assert list(s.topics["/a"].rate_history) == [2.0]

    def test_next_period_adds_a_sample(self):
        s = StatsState()
        s.apply(self.w(100.0, 2.0))
        s.apply(self.w(105.0, 3.0))
        assert list(s.topics["/a"].rate_history) == [2.0, 3.0]


class TestPerSideMerge:
    # /tf_static is published by one process and received by twenty. Each
    # process's window carries only its own side; a recv-only window must not
    # erase the pub rate learned from the publisher's window.
    def test_recv_only_window_keeps_pub_fields(self):
        s = StatsState()
        s.apply(parse_jsonl_line(
            '{"ts_ns":"1000000000","window_s":5.0,"topics":[{"topic":"/tf","pub_inter_hz":2.0,"pub_intra_hz":0.0}],"nodes":[],"warns":[]}'))
        s.apply(parse_jsonl_line(
            '{"ts_ns":"1100000000","window_s":5.0,"topics":[{"topic":"/tf","recv_inter_hz":2.0,"recv_intra_hz":0.0,"recv_endpoint_seen":true}],"nodes":[],"warns":[]}'))
        t = s.topics["/tf"].latest
        assert t.pub_inter_hz == 2.0
        assert t.recv_inter_hz == 2.0
        assert t.recv_endpoint_seen is True

    def test_pub_side_refresh_overwrites_pub_only(self):
        s = StatsState()
        s.apply(parse_jsonl_line(
            '{"ts_ns":"1000000000","window_s":5.0,"topics":[{"topic":"/tf","recv_inter_hz":9.0,"recv_max_dt_ms":50.0}],"nodes":[],"warns":[]}'))
        s.apply(parse_jsonl_line(
            '{"ts_ns":"1100000000","window_s":5.0,"topics":[{"topic":"/tf","pub_inter_hz":2.0,"pub_max_dt_ms":500.0}],"nodes":[],"warns":[]}'))
        t = s.topics["/tf"].latest
        assert (t.pub_inter_hz, t.pub_max_dt_ms) == (2.0, 500.0)
        assert (t.recv_inter_hz, t.recv_max_dt_ms) == (9.0, 50.0)


class TestWarnRetention:
    def test_transient_warn_survives_in_recent_with_age_in_seconds(self):
        # Review #32: warns replaced wholesale each window made a one-window gap
        # transient a sub-second blink. recent_warns retains it with its age,
        # in seconds, since window counts mean nothing across processes.
        s = StatsState()
        s.apply(parse_jsonl_line(RECORD))          # 2 warns fire at ts 1782887153.899
        s.apply(parse_jsonl_line(
            '{"ts_ns":"1782887160899445923","window_s":5.0,'
            '"topics":[{"topic":"/scan","pub_inter_hz":20.0}],"nodes":["/perception"],"warns":[]}'))
        assert s.warns == []                       # live set is empty
        kinds = [w.kind for _, w in s.recent_warns]
        assert "topic_gap" in kinds and "node_missing" in kinds
        ages = [s.warn_age_s(ts) for ts, _ in s.recent_warns]
        assert ages == pytest.approx([7.0, 7.0])
