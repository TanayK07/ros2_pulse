"""The demo is the README GIF. Its incidents must be unambiguous on screen."""

from pulse_top.demo import _window
from pulse_top.model import parse_jsonl_line
import json


def test_cmd_vel_sag_is_below_its_min_in_every_sagging_window():
    # Frame 11 of the first GIF cut read "topic_rate /cmd_vel 20.04Hz > max 22.0Hz":
    # the sag overlapped the min bound, so the warn fired on an in-range rate.
    for i in range(8, 15):
        w = _window(i)
        warn = next(x for x in w["warns"] if x["kind"] == "topic_rate")
        assert warn["hz"] < warn["min_hz"], (i, warn)
