"""FileFollower tests: tail-follow with rotation/truncation, partial-line safety.

The probe appends whole '\n'-terminated records, but the follower can wake mid-write:
a partial trailing line must be held back, never parsed, and delivered once its
newline lands. Truncation (log rotation, probe restart with O_TRUNC) must re-sync
instead of replaying stale bytes or dying.
"""

from pulse_top.reader import FileFollower


def write(path, data, mode="a"):
    with open(path, mode) as f:
        f.write(data)


class TestFollow:
    def test_reads_appended_lines(self, tmp_path):
        p = tmp_path / "log"
        write(p, "one\ntwo\n", "w")
        f = FileFollower(str(p))
        assert f.poll() == ["one", "two"]
        write(p, "three\n")
        assert f.poll() == ["three"]
        assert f.poll() == []

    def test_partial_line_held_until_complete(self, tmp_path):
        p = tmp_path / "log"
        write(p, '{"ts_ns":"1"', "w")
        f = FileFollower(str(p))
        assert f.poll() == []                 # incomplete record: not delivered
        write(p, ',"window_s":1.0}\n')
        assert f.poll() == ['{"ts_ns":"1","window_s":1.0}']

    def test_truncation_resyncs(self, tmp_path):
        p = tmp_path / "log"
        write(p, "old-window-1\nold-window-2\n", "w")
        f = FileFollower(str(p))
        f.poll()
        write(p, "fresh\n", "w")              # probe restarted: file truncated
        assert f.poll() == ["fresh"]

    def test_missing_file_is_quiet_then_attaches(self, tmp_path):
        p = tmp_path / "not-yet"
        f = FileFollower(str(p))
        assert f.poll() == []                 # no file yet: no crash, no lines
        write(p, "late\n", "w")
        assert f.poll() == ["late"]
