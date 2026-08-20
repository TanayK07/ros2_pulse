"""Tail-follow a probe log without inotify: poll, hold partial lines, survive rotation.

The probe writes whole '\n'-terminated records with a single write() per window, but
this poller can still land mid-write; the trailing partial line is buffered until its
newline arrives. Truncation (probe restart, logrotate copytruncate) is detected by
the file shrinking below our offset — re-sync to the start rather than replaying or
dying. A missing file is quiet: the probe may simply not have started yet.
"""

from __future__ import annotations

import os


class FileFollower:
    def __init__(self, path: str):
        self.path = path
        self._offset = 0
        self._partial = ""

    def poll(self) -> list[str]:
        """Return complete new lines since the last poll (without newlines)."""
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if size < self._offset:  # truncated/rotated: start over
            self._offset = 0
            self._partial = ""
        if size == self._offset:
            return []
        try:
            with open(self.path, "r", errors="replace") as f:
                f.seek(self._offset)
                chunk = f.read()
                self._offset = f.tell()
        except OSError:
            return []
        data = self._partial + chunk
        *lines, self._partial = data.split("\n")
        return [ln for ln in lines if ln]
