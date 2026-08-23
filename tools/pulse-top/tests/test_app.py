"""View-layer tests, headless via Textual's pilot.

Pinned here: staleness is rendered in seconds (not windows), and a tick that
changes nothing on screen issues no DataTable cell updates, update_cell
invalidates the table's row render caches and schedules a refresh
unconditionally (Textual 8.2 _data_table.py), so 240 no-op calls per tick
repainted the whole table twice a second over ssh (Orin, 2026-08-23).
"""

import asyncio

import pytest
from textual.widgets import DataTable

from pulse_top.app import PulseTopApp


def window(ts_s, topics, window_s=5.0):
    tt = ",".join(f'{{"topic":"{n}","pub_inter_hz":{hz}}}' for n, hz in topics)
    return f'{{"ts_ns":"{int(ts_s * 1e9)}","window_s":{window_s},"topics":[{tt}],"nodes":[],"warns":[]}}\n'


@pytest.fixture
def log(tmp_path):
    p = tmp_path / "topic_freq.1.log"
    p.write_text("")
    return p


async def settle(pilot, n=3):
    for _ in range(n):
        await pilot.pause(0.08)


class TestStaleRendering:
    async def test_stale_row_shows_age_in_seconds(self, log):
        app = PulseTopApp(str(log), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            with open(log, "a") as f:
                f.write(window(100.0, [("/a", 5.0)]))
                f.write(window(112.0, [("/b", 1.0)]))   # /a is now 12 s old
            await settle(pilot)
            table = app.query_one("#table", DataTable)
            name_cell = table.get_cell("/a", "TOPIC")
            assert "stale 12s" in name_cell.plain
            assert "w" not in name_cell.plain.split("stale")[1]
            assert "stale" not in table.get_cell("/b", "TOPIC").plain


class TestRepaintEconomy:
    async def test_unchanged_rows_issue_no_cell_updates(self, log, monkeypatch):
        calls = []
        real = DataTable.update_cell

        def counting(self, *a, **kw):
            calls.append(a[:2])
            return real(self, *a, **kw)

        monkeypatch.setattr(DataTable, "update_cell", counting)
        app = PulseTopApp(str(log), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            with open(log, "a") as f:
                f.write(window(100.0, [("/a", 5.0), ("/b", 1.0)]))
            await settle(pilot)
            calls.clear()
            # Another process's window lands: same topics, same rates, 0.1 s later.
            with open(log, "a") as f:
                f.write(window(100.1, [("/a", 5.0), ("/b", 1.0)]))
            await settle(pilot)
            assert calls == []
            # One rate changes: only that row's changed cells are touched.
            with open(log, "a") as f:
                f.write(window(100.2, [("/a", 7.0), ("/b", 1.0)]))
            await settle(pilot)
            rows = {row for row, _ in calls}
            assert rows == {"/a"}
            assert 0 < len(calls) < 6


def test_window_title_is_the_product_name():
    assert PulseTopApp.TITLE == "pulse-top"
