"""The probe's default TEXT format, read as well as jsonl (issue #58).

The probe writes text unless ROS_TOPIC_STATS_FORMAT=jsonl, and a stack set up from a
.bashrc rarely sets it. pulse-top used to parse jsonl only, so a directory full of
healthy text logs rendered as "0 file(s)" forever. The text format is pinned
byte-for-byte by the probe's own tests (src/core/window_format.cpp, read by the C++
log_reader that pulse-check uses), so it is a stable contract to parse.

The strongest pin: the SAME window in both encodings must produce the SAME Window.
"""

from pulse_top.model import LogParser, parse_jsonl_line

JSONL = (
    '{"ts_ns":"1782887153899445923","window_s":5.000,'
    '"topics":[{"topic":"/scan","pub_inter_hz":19.800000,"pub_intra_hz":0.000000,'
    '"recv_inter_hz":19.800000,"recv_intra_hz":0.000000,"recv_endpoint_seen":true,'
    '"pub_max_dt_ms":812.400},'
    '{"topic":"/points","pub_inter_hz":10.000000,"pub_intra_hz":10.000000},'
    '{"topic":"/cmd","recv_inter_hz":0.000000,"recv_intra_hz":0.000000,"recv_endpoint_seen":true,'
    '"recv_max_dt_ms":1500.000}],'
    '"nodes":["/perception","/planner"],'
    '"warns":[{"kind":"topic_gap","topic":"/scan","max_dt_ms":812.400,"max_gap_ms":250},'
    '{"kind":"topic_rate","topic":"/points","hz":10.000000,"min_hz":19.8},'
    '{"kind":"topic_rate","topic":"/cmd","hz":0.000000,"min_hz":0,"max_hz":5},'
    '{"kind":"node_missing","node":"/localization"}]}'
)

# The same window as the probe's text emitter writes it (formatWindow + renderWarnLine).
TEXT = """\
# ts_ns=1782887153899445923 window_s=5.000
TOPIC /scan 19.800000
JITTER /scan pub max_dt_ms=812.400
RECV /scan inter=19.800000 intra=0.000000
TOPIC /points 10.000000
PUB /points inter=10.000000 intra=10.000000
RECV /cmd inter=0.000000 intra=0.000000
JITTER /cmd recv max_dt_ms=1500.000
NODE /perception
NODE /planner
WARN TOPIC /scan max_dt_ms=812.400 expected_max_gap_ms=250
WARN TOPIC /points hz=10.000000 expected=[19.8,inf]
WARN TOPIC /cmd hz=0.000000 expected=[0,5]
WARN NODE /localization missing

"""


def feed_all(parser, text):
    out = []
    for line in text.split("\n"):
        out.extend(parser.feed(line))
    return out


class TestTextFormat:
    def test_text_window_equals_its_jsonl_twin(self):
        windows = feed_all(LogParser(), TEXT)
        assert windows == [parse_jsonl_line(JSONL)]

    def test_real_probe_output_humble(self):
        # Captured from libros2_pulse.so preloaded into `ros2 topic pub` on Humble,
        # default format, ROS_TOPIC_STATS_JITTER=1, plus the ros2cli daemon's file.
        text = (
            "# ts_ns=1790937483279862991 window_s=5.000\n"
            "TOPIC /chatter 8.999742\n"
            "JITTER /chatter pub max_dt_ms=101.228\n"
            "NODE /_ros2cli_10736\n"
            "\n"
            "# ts_ns=1790937484841995811 window_s=1.562\n"
            "TOPIC /chatter 9.601895\n"
            "JITTER /chatter pub max_dt_ms=100.140\n"
            "NODE /_ros2cli_10736\n"
            "\n"
        )
        ws = feed_all(LogParser(), text)
        assert [w.ts_ns for w in ws] == [1790937483279862991, 1790937484841995811]
        assert ws[0].window_s == 5.0
        t = ws[0].topics[0]
        assert (t.topic, t.pub_inter_hz, t.pub_intra_hz, t.pub_max_dt_ms) == ("/chatter", 8.999742, 0.0, 101.228)
        assert t.recv_inter_hz is None and t.recv_endpoint_seen is False  # absent, not zero
        assert ws[1].nodes == ["/_ros2cli_10736"]

    def test_block_is_emitted_on_its_blank_line_not_one_window_late(self):
        p = LogParser()
        assert p.feed("# ts_ns=1000000000 window_s=1.000") == []
        assert p.feed("TOPIC /a 5.000000") == []
        (w,) = p.feed("")
        assert w.topics[0].pub_inter_hz == 5.0

    def test_header_without_blank_line_closes_the_previous_block(self):
        p = LogParser()
        p.feed("# ts_ns=1000000000 window_s=1.000")
        p.feed("TOPIC /a 5.000000")
        (w,) = p.feed("# ts_ns=2000000000 window_s=1.000")
        assert w.ts_ns == 1_000_000_000

    def test_empty_window_still_counts(self):
        # An idle process (the ros2cli daemon) writes header + NODE only: still a window.
        (w,) = feed_all(LogParser(), "# ts_ns=5 window_s=5.000\nNODE /_ros2cli_daemon_0_ab\n\n")
        assert w.topics == [] and w.nodes == ["/_ros2cli_daemon_0_ab"]

    def test_pre_header_noise_and_attach_mid_block_are_ignored(self):
        p = LogParser()
        # attached mid-block (FileFollower tail seek): orphan lines before a header
        assert feed_all(p, "TOPIC /a 5.000000\nNODE /x\n\n") == []
        assert p.unrecognised == 0  # probe lines, just orphaned: not "foreign"

    def test_malformed_lines_never_raise(self):
        p = LogParser()
        ws = feed_all(p, "# ts_ns=1 window_s=1.000\nTOPIC /a notanumber\nRECV /b inter=x\nTOPIC\n\n")
        assert len(ws) == 1 and ws[0].topics == []

    def test_jsonl_lines_pass_through_and_mixed_files_work(self):
        # Probe restarted with the other format into a shared OUTPUT_FILE.
        p = LogParser()
        ws = feed_all(p, TEXT + JSONL + "\n")
        assert len(ws) == 2 and ws[0] == ws[1]

    def test_foreign_lines_are_counted(self):
        p = LogParser()
        feed_all(p, "hello\nworld\n")
        assert p.unrecognised == 2
        assert p.windows == 0
