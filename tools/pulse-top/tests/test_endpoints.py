"""Per-endpoint split (probe 0.7+): jsonl "endpoints" and the opt-in text EP lines.

Additive in both formats: the topic rows are unchanged, an EP line is a recognised
probe line (never counted as foreign), and a record without the key parses to an
empty list.
"""

import json

from pulse_top.model import EndpointWindow, LogParser, parse_jsonl_line

TEXT = """\
# ts_ns=1782887153000000000 window_s=5.000
TOPIC /scan 20.000000
NODE /lidar
EP /scan pub id=3 node=/lidar inter=20.000000 intra=0.000000 max_dt_ms=52.500
EP /scan recv id=7 inter=0.000000 intra=10.000000 loaned_hz=1.000000

"""

JSONL = json.dumps({"ts_ns": "1782887153000000000", "window_s": 5.0, "topics": [
    {"topic": "/scan", "pub_inter_hz": 20.0, "pub_intra_hz": 0.0}],
    "nodes": ["/lidar"], "warns": [], "endpoints": [
    {"topic": "/scan", "side": "pub", "id": 3, "node": "/lidar", "inter_hz": 20.0,
     "intra_hz": 0.0, "max_dt_ms": 52.5},
    {"topic": "/scan", "side": "recv", "id": 7, "inter_hz": 0.0, "intra_hz": 10.0,
     "loaned_hz": 1.0}]})

WANT = [
    EndpointWindow(topic="/scan", side="pub", id=3, node="/lidar", inter_hz=20.0,
                   intra_hz=0.0, max_dt_ms=52.5, loaned_hz=None),
    EndpointWindow(topic="/scan", side="recv", id=7, node=None, inter_hz=0.0,
                   intra_hz=10.0, max_dt_ms=None, loaned_hz=1.0),
]


def test_jsonl_endpoints_parsed():
    w = parse_jsonl_line(JSONL)
    assert w.endpoints == WANT


def test_text_ep_lines_parse_like_jsonl_twin_and_are_recognised():
    p = LogParser()
    wins = []
    for line in TEXT.splitlines(keepends=True):
        wins.extend(p.feed(line))
    assert p.unrecognised == 0
    [w] = wins
    assert w.endpoints == WANT
    assert w == parse_jsonl_line(JSONL)


def test_record_without_endpoints_is_empty_list():
    w = parse_jsonl_line(json.dumps({"ts_ns": "1", "window_s": 1.0, "topics": [],
                                     "nodes": [], "warns": []}))
    assert w.endpoints == []


def test_unknown_side_skipped_not_fatal():
    w = parse_jsonl_line(json.dumps({"ts_ns": "1", "window_s": 1.0, "topics": [],
                                     "nodes": [], "warns": [], "endpoints": [
                                         {"topic": "/x", "side": "service", "id": 1}]}))
    assert w is not None
    assert w.endpoints == []


def _rec(endpoints):
    return json.dumps({"ts_ns": "1", "window_s": 1.0,
                       "topics": [{"topic": "/scan", "pub_inter_hz": 20.0}],
                       "nodes": ["/lidar"], "warns": [], "endpoints": endpoints})


GOOD = {"topic": "/scan", "side": "pub", "id": 3, "node": "/lidar", "inter_hz": 20.0,
        "intra_hz": 0.0}


def test_element_missing_topic_costs_that_element_not_the_window():
    """Like the C++ reader: one bad entry is dropped, the topics/nodes/warns survive."""
    w = parse_jsonl_line(_rec([{"side": "recv", "id": 1, "inter_hz": 1.0}, GOOD]))
    assert w is not None
    assert [t.topic for t in w.topics] == ["/scan"]
    assert w.nodes == ["/lidar"]
    assert [e.id for e in w.endpoints] == [3]


def test_non_object_and_malformed_elements_never_raise():
    """A non-dict element used to raise AttributeError out of parse_jsonl_line, whose
    contract is "never an exception": it would kill the live TUI."""
    bad = [1, "x", None, [], {"topic": 5, "side": "pub"},
           {"topic": "/a", "side": "pub", "inter_hz": "fast"},
           {"topic": "/a", "side": "pub", "node": 7},
           {"topic": "/a", "side": "recv", "max_dt_ms": {"ms": 1}},
           {"topic": "/a", "side": "recv", "id": "seven"}]
    w = parse_jsonl_line(_rec(bad + [GOOD]))
    assert w is not None
    assert [e.id for e in w.endpoints] == [3]
    for raw in (1, "x", {"a": 1}, None):  # a non-array "endpoints" reads as empty
        w = parse_jsonl_line(_rec(raw))
        assert w is not None and w.endpoints == [], raw


def test_logparser_counts_record_with_bad_endpoint_as_window():
    p = LogParser()
    wins = p.feed(_rec([1, GOOD]) + "\n")
    assert len(wins) == 1 and p.unrecognised == 0


def test_endpoint_distributions_are_ignored():
    # A probe addition pulse-top does not use: a subscription's own receive distributions.
    # The record must parse exactly as without them.
    rec = json.loads(JSONL)
    rec["endpoints"][1].update({"avg_dt_ms": 100.0, "min_dt_ms": 98.5, "stddev_dt_ms": 0.5,
                                "dt_count": 49, "age_avg_ms": 1.25, "age_min_ms": -0.5,
                                "age_max_ms": 3.0, "age_stddev_ms": 0.4, "age_count": 50})
    assert parse_jsonl_line(json.dumps(rec)) == parse_jsonl_line(JSONL)


def test_partial_flag_is_parsed_and_changes_nothing_else():
    # The probe flags its attach and atexit windows "partial", absent on a full one.
    # pulse-export forgets a process on its exit window. The text format has no flag.
    rec = json.loads(JSONL)
    rec["partial"] = True
    w = parse_jsonl_line(json.dumps(rec))
    assert w.partial is True
    w.partial = False
    assert w == parse_jsonl_line(JSONL)
    rec["partial"] = "true"  # only a JSON true counts
    assert parse_jsonl_line(json.dumps(rec)).partial is False
