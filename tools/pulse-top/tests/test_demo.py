"""The demo is the README GIF. Its incidents must be unambiguous on screen."""

from pulse_top.demo import _window
from pulse_top.model import StatsState, parse_jsonl_line
import json


def test_camera_recv_lag_is_derived_then_clears():
    # Issue #50: the demo scripts a /camera/image_raw receive sag with no probe
    # warn for it; the model must derive recv_lag from the rates alone and drop
    # it again once the sag ends, so the GIF shows both edges.
    s = StatsState()
    seen, cleared = False, False
    for i in range(48):
        s.apply(parse_jsonl_line(json.dumps(_window(i))))
        lag = [w for w in s.warns if w.kind == "recv_lag"]
        if lag:
            assert [w.topic for w in lag] == ["/camera/image_raw"]
            seen = True
        elif seen:
            cleared = True
    assert seen and cleared


def test_cmd_vel_sag_is_below_its_min_in_every_sagging_window():
    # Frame 11 of the first GIF cut read "topic_rate /cmd_vel 20.04Hz > max 22.0Hz":
    # the sag overlapped the min bound, so the warn fired on an in-range rate.
    for i in range(8, 15):
        w = _window(i)
        warn = next(x for x in w["warns"] if x["kind"] == "topic_rate")
        assert warn["hz"] < warn["min_hz"], (i, warn)
