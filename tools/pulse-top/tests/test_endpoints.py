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
