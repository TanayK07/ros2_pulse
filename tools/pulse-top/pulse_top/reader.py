"""Tail-follow a probe log without inotify: poll, hold partial lines, survive rotation.

Byte-exact by construction (PR #32 review): the file is read in BINARY and offsets
are byte offsets, so `os.path.getsize()` comparisons are sound and a poll landing
mid-UTF-8-sequence holds the partial bytes with the partial line instead of decoding
them to U+FFFD. Decoding happens per completed line only.

Attach behavior (PR #32 review): a first poll on a large backlog seeks to the tail
(TAIL_BYTES before EOF, dropping the mid-record line it lands on) and every poll
reads at most READ_CAP bytes, carrying the rest to the next tick, the UI thread
never blocks on hours of history it will only fold into a 60-window deque.

Truncation (probe restart, logrotate copytruncate) is detected by the file shrinking
below our offset, re-sync to the start rather than replaying or dying. A missing
file is quiet: the probe may simply not have started yet.
"""

from __future__ import annotations

import glob
import os


class FileFollower:
    TAIL_BYTES = 256 * 1024   # backlog beyond this: attach at tail, not history
    READ_CAP = 1024 * 1024    # max bytes consumed per poll; rest carries over

    def __init__(self, path: str):
        self.path = path
        self._offset = 0
        self._partial = b""
        self._attached = False
        self._skip_partial = False

    def poll(self) -> list[str]:
        """Return complete new lines since the last poll (without newlines)."""
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if not self._attached:
            self._attached = True
            if size > self.TAIL_BYTES:
                self._offset = size - self.TAIL_BYTES
                self._skip_partial = True  # we land mid-record; drop that line
        if size < self._offset:  # truncated/rotated: start over
            self._offset = 0
            self._partial = b""
            self._skip_partial = False
        if size == self._offset:
            return []
        try:
            with open(self.path, "rb") as f:
                f.seek(self._offset)
                chunk = f.read(self.READ_CAP)
                self._offset = f.tell()
        except OSError:
            return []
        data = self._partial + chunk
        *lines, self._partial = data.split(b"\n")
        if self._skip_partial and lines:
            lines = lines[1:]
            self._skip_partial = False
        return [ln.decode("utf-8", errors="replace") for ln in lines if ln]


class MultiFollower:
    """Follow every file matching a glob pattern (or one plain path).

    The probe's default output is one file per process, $TMPDIR/topic_freq.<pid>.log,
    so a live stack is N files that appear as nodes start. Re-glob on every poll,
    a directory listing twice a second is nothing next to the cost of showing one
    node of a 77-node graph. A plain path (no glob metacharacters) is followed
    as-is so a deliberately shared file keeps working.
    """

    def __init__(self, pattern: str):
        self.pattern = pattern
        self._followers: dict[str, FileFollower] = {}

    @property
    def files(self) -> list[str]:
        return sorted(self._followers)

    def poll(self) -> list[str]:
        if glob.has_magic(self.pattern):
            for path in glob.glob(self.pattern):
                if path not in self._followers:
                    self._followers[path] = FileFollower(path)
        elif not self._followers:
            self._followers[self.pattern] = FileFollower(self.pattern)
        out: list[str] = []
        for path in self.files:
            out.extend(self._followers[path].poll())
        return out
