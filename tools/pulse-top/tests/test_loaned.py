"""Loaned-message rates (rclcpp#3153 follow-up): pub_loaned_hz / recv_loaned_hz.

The probe counts real middleware loans by wrapping rcl_publish_loaned_message and
rcl_take_loaned_message. They are SUBSETS of the publish / receive totals and, like
every optional per-topic field, absent when zero: an absent loaned rate must stay None
here and must not become a 0 series in the exporter.
"""

import json

from pulse_top.export import Exporter, render_prometheus
from pulse_top.model import LogParser, StatsState, parse_jsonl_line

TS = 1782887153000000000

JSONL_PUB = json.dumps({"ts_ns": str(TS), "window_s": 2.0, "topics": [
    {"topic": "/cloud", "pub_inter_hz": 50.0, "pub_intra_hz": 0.0, "pub_loaned_hz": 50.0}],
    "nodes": ["/talker"], "warns": []})
JSONL_SUB = json.dumps({"ts_ns": str(TS + 1), "window_s": 2.0, "topics": [
    {"topic": "/cloud", "recv_inter_hz": 49.0, "recv_intra_hz": 0.0,
     "recv_endpoint_seen": True, "recv_loaned_hz": 49.0}],
    "nodes": ["/listener"], "warns": []})

TEXT = """\
# ts_ns=1782887153000000000 window_s=2.000
TOPIC /cloud 50.000000
LOAN /cloud pub hz=50.000000
RECV /cloud inter=49.000000 intra=0.000000
LOAN /cloud recv hz=49.000000
NODE /talker

"""

JSONL_BOTH = (
    '{"ts_ns":"1782887153000000000","window_s":2.000,"topics":[{"topic":"/cloud",'
    '"pub_inter_hz":50.000000,"pub_intra_hz":0.000000,"recv_inter_hz":49.000000,'
    '"recv_intra_hz":0.000000,"recv_endpoint_seen":true,"pub_loaned_hz":50.000000,'
    '"recv_loaned_hz":49.000000}],"nodes":["/talker"],"warns":[]}'
)


def _text_windows(text):
    p = LogParser()
    out = []
    for line in text.splitlines(keepends=True):
        out.extend(p.feed(line))
    return out


def test_jsonl_loaned_fields_parsed():
    w = parse_jsonl_line(JSONL_PUB)
    assert w.topics[0].pub_loaned_hz == 50.0
    assert w.topics[0].recv_loaned_hz is None  # absent stays absent


def test_absent_loaned_stays_none():
    w = parse_jsonl_line(json.dumps({"ts_ns": "1", "window_s": 1.0, "topics": [
        {"topic": "/x", "pub_inter_hz": 5.0, "pub_intra_hz": 0.0}], "nodes": [], "warns": []}))
    assert w.topics[0].pub_loaned_hz is None
    assert w.topics[0].recv_loaned_hz is None


def test_text_loan_lines_parse_like_jsonl_twin():
    [tw] = _text_windows(TEXT)
    [jw] = [parse_jsonl_line(JSONL_BOTH)]
    assert tw == jw
    assert tw.topics[0].pub_loaned_hz == 50.0
    assert tw.topics[0].recv_loaned_hz == 49.0


def test_loan_line_is_not_unrecognised():
    p = LogParser()
    for line in TEXT.splitlines(keepends=True):
        p.feed(line)
    assert p.unrecognised == 0


def test_merge_keeps_each_side_loaned_rate():
    st = StatsState()
    st.apply(parse_jsonl_line(JSONL_PUB), source="/tmp/topic_freq.1.log")
    st.apply(parse_jsonl_line(JSONL_SUB), source="/tmp/topic_freq.2.log")
    t = st.topics["/cloud"].latest
    assert (t.pub_loaned_hz, t.recv_loaned_hz) == (50.0, 49.0)


def test_exporter_loaned_series_only_when_measured():
    ex = Exporter(poll_s=1.0, clock=lambda: 1000.0)
    ex.ingest("/tmp/topic_freq.4242.log", JSONL_PUB)
    ex.ingest("/tmp/topic_freq.5151.log", JSONL_SUB)
    text = render_prometheus(ex.collect())
    assert 'ros2_pulse_topic_loaned_rate_hertz{pid="4242",topic="/cloud",side="pub"} 50' in text
    assert 'ros2_pulse_topic_loaned_rate_hertz{pid="5151",topic="/cloud",side="recv"} 49' in text
    assert 'loaned_rate_hertz{pid="4242",topic="/cloud",side="recv"}' not in text
    assert 'loaned_rate_hertz{pid="5151",topic="/cloud",side="pub"}' not in text
