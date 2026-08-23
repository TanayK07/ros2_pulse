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


class TestByteExactness:
    def test_partial_multibyte_char_held_not_mangled(self, tmp_path):
        # Review #32: text-mode read with errors="replace" turned a split UTF-8
        # sequence into U+FFFD and desynced the offset. Binary offsets must hold
        # the partial BYTES with the partial line.
        p = tmp_path / "log"
        with open(p, "wb") as f:
            f.write(b'{"t":"/\xce')          # first byte of two-byte alpha, no newline
        f2 = FileFollower(str(p))
        assert f2.poll() == []
        with open(p, "ab") as f:
            f.write(b'\xb1"}\n')
        assert f2.poll() == ['{"t":"/α"}']

    def test_large_backlog_attach_seeks_to_tail(self, tmp_path):
        # Review #32: attaching to a long-running log must not replay the whole
        # backlog through the UI tick. First poll on a big file starts near EOF
        # and drops the mid-record partial it lands on.
        p = tmp_path / "log"
        filler = ("x" * 99 + "\n") * 4000     # 400 KB > TAIL_BYTES
        write(p, filler + "last-line\n", "w")
        f = FileFollower(str(p))
        lines = f.poll()
        assert lines, "tail attach must still deliver recent lines"
        assert lines[-1] == "last-line"
        assert len(lines) < 4001              # did NOT replay the whole backlog

    def test_read_cap_carries_remainder_to_next_poll(self, tmp_path):
        p = tmp_path / "log"
        write(p, "a\nb\n", "w")
        f = FileFollower(str(p))
        f.READ_CAP = 2                        # force two polls for two lines
        assert f.poll() == ["a"]
        assert f.poll() == ["b"]


class TestMultiFollower:
    # The probe writes one file per process by default ($TMPDIR/topic_freq.<pid>.log).
    # A dashboard that follows only the newest file shows one node of a 77-node
    # stack. Follow every file matching the pattern, and notice new ones.
    def test_reads_lines_from_every_matching_file(self, tmp_path):
        from pulse_top.reader import MultiFollower
        (tmp_path / "topic_freq.1.log").write_text("one\n")
        (tmp_path / "topic_freq.2.log").write_text("two\n")
        (tmp_path / "other.log").write_text("nope\n")
        mf = MultiFollower(str(tmp_path / "topic_freq.*.log"))
        assert sorted(mf.poll()) == ["one", "two"]
        assert len(mf.files) == 2

    def test_picks_up_a_file_created_after_start(self, tmp_path):
        from pulse_top.reader import MultiFollower
        (tmp_path / "topic_freq.1.log").write_text("one\n")
        mf = MultiFollower(str(tmp_path / "topic_freq.*.log"))
        assert mf.poll() == ["one"]
        (tmp_path / "topic_freq.2.log").write_text("two\n")
        assert mf.poll() == ["two"]
        assert len(mf.files) == 2

    def test_single_plain_path_still_works(self, tmp_path):
        from pulse_top.reader import MultiFollower
        f = tmp_path / "shared.log"
        f.write_text("a\n")
        mf = MultiFollower(str(f))
        assert mf.poll() == ["a"]
        with open(f, "a") as fh:
            fh.write("b\n")
        assert mf.poll() == ["b"]
