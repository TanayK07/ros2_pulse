"""View-layer tests, headless via Textual's pilot.

Pinned here: staleness is rendered in seconds (not windows), and a tick that
changes nothing on screen issues no DataTable cell updates, update_cell
invalidates the table's row render caches and schedules a refresh
unconditionally (Textual 8.2 _data_table.py), so 240 no-op calls per tick
repainted the whole table twice a second over ssh (Orin, 2026-08-23).
"""

import asyncio

import pytest
from textual.widgets import DataTable, Static

from pulse_top.app import WARN, PulseTopApp


def window(ts_s, topics, window_s=5.0):
    tt = ",".join(f'{{"topic":"{n}","pub_inter_hz":{hz}}}' for n, hz in topics)
    return f'{{"ts_ns":"{int(ts_s * 1e9)}","window_s":{window_s},"topics":[{tt}],"nodes":[],"warns":[]}}\n'


def recv_window(ts_s, topic, hz, window_s=1.0):
    return (f'{{"ts_ns":"{int(ts_s * 1e9)}","window_s":{window_s},'
            f'"topics":[{{"topic":"{topic}","recv_inter_hz":{hz},"recv_intra_hz":0.0,'
            f'"recv_endpoint_seen":true}}],"nodes":[],"warns":[]}}\n')


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


class TestRecvLag:
    # Issue #50: the publisher's and the subscriber's processes write separate
    # files. A backlog is read file by file, so the app must order a poll's
    # windows by ts_ns before applying them or the pub/recv pairing sees the
    # publisher's whole file before the subscriber's first window.
    async def test_recv_lag_is_live_then_ages(self, tmp_path):
        pub = tmp_path / "topic_freq.1.log"
        sub = tmp_path / "topic_freq.2.log"
        with open(pub, "w") as fp, open(sub, "w") as fs:
            for i in range(3):
                fp.write(window(100.0 + i, [("/scan", 20.0)], window_s=1.0))
                fs.write(recv_window(100.1 + i, "/scan", 12.0))
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            tab = app.query_one("#warns-list", Static).content.plain
            assert tab.startswith("recv_lag") and "ago" not in tab
            table = app.query_one("#table", DataTable)
            assert WARN in str(table.get_cell("/scan", "RECV").style)
            sidebar = app.query_one("#sidebar", Static).content.plain
            assert "recv lag" in sidebar and "-40% over 3w" in sidebar
            with open(pub, "a") as fp, open(sub, "a") as fs:
                fp.write(window(103.0, [("/scan", 20.0)], window_s=1.0))
                fs.write(recv_window(103.1, "/scan", 19.8))     # recovered
            await settle(pilot)
            tab = app.query_one("#warns-list", Static).content.plain
            assert tab.startswith("recv_lag") and tab.endswith("ago")
            assert WARN not in str(table.get_cell("/scan", "RECV").style)

    async def test_frozen_episode_with_equal_numbers_renders_as_history(self, tmp_path):
        # Review of #55: liveness used `w in warns` (dataclass equality), so a
        # cleared recv_lag whose numbers match the current live one rendered live.
        pub = tmp_path / "topic_freq.1.log"
        sub = tmp_path / "topic_freq.2.log"
        with open(pub, "w") as fp, open(sub, "w") as fs:
            for i in (0, 1, 2, 4, 5, 6):                        # two identical episodes
                fp.write(window(100.0 + i, [("/scan", 20.0)], window_s=1.0))
                fs.write(recv_window(100.1 + i, "/scan", 12.0))
                if i == 2:
                    fp.write(window(103.0, [("/scan", 20.0)], window_s=1.0))
                    fs.write(recv_window(103.1, "/scan", 19.8))   # recovered in between
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            lines = [ln for ln in app.query_one("#warns-list", Static).content.plain.splitlines()
                     if ln.startswith("recv_lag")]
            assert len(lines) == 2
            assert not lines[0].endswith("ago") and lines[1].endswith("ago")

    async def test_sidebar_shows_worst_deficit_and_source_count(self, tmp_path):
        pub = tmp_path / "topic_freq.1.log"
        a, b = tmp_path / "topic_freq.2.log", tmp_path / "topic_freq.3.log"
        with open(pub, "w") as fp, open(a, "w") as fa, open(b, "w") as fb:
            for i in range(3):
                fp.write(window(100.0 + i, [("/scan", 20.0)], window_s=1.0))
                fa.write(recv_window(100.1 + i, "/scan", 16.0))
                fb.write(recv_window(100.2 + i, "/scan", 12.0))
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            sidebar = app.query_one("#sidebar", Static).content.plain
            assert "-40% over 3w (2 sources)" in sidebar
            assert "topic_freq.3.log" in sidebar

    def test_lag_flags_reach_the_model(self, tmp_path):
        app = PulseTopApp(str(tmp_path / "x.log"), lag_tol=0.2, lag_windows=5)
        assert (app._state.lag_tol, app._state.lag_windows) == (0.2, 5)


def test_window_title_is_the_product_name():
    assert PulseTopApp.TITLE == "pulse-top"


def topbar(app) -> str:
    return str(app.query_one("#topbar", Static).render())


def notice(app) -> str:
    n = app.query_one("#notice", Static)
    return str(n.render()) if n.display else ""


TEXT_BLOCK = ("# ts_ns={ts} window_s=1.000\n"
              "TOPIC /chatter 9.601895\n"
              "JITTER /chatter pub max_dt_ms=100.140\n"
              "NODE /_ros2cli_{pid}\n"
              "\n")


class TestIssue58FilesFound:
    # Issue #58: probe in .bashrc, default text format, many files. pulse-top said
    # "0 file(s)" forever: text lines never parsed, and the top bar was only redrawn
    # after a parsed window, so even the file count it had was never shown.
    async def test_text_format_logs_render(self, tmp_path):
        for pid in (101, 102):
            (tmp_path / f"topic_freq.{pid}.log").write_text(
                TEXT_BLOCK.format(ts=100_000_000_000 + pid, pid=pid))
        (tmp_path / "topic_freq.103.log").write_text("")  # short-lived process, nothing yet
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            assert "3 file(s)" in topbar(app)
            assert "2 windows" in topbar(app)
            table = app.query_one("#table", DataTable)
            assert table.get_cell("/chatter", "PUB Hz").plain == "9.6"
            assert notice(app) == ""

    async def test_file_count_shows_before_any_window(self, tmp_path):
        (tmp_path / "topic_freq.1.log").write_text("")
        (tmp_path / "topic_freq.2.log").write_text("")
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            assert "2 file(s)" in topbar(app)
            assert "waiting for the first window" in notice(app)

    async def test_files_created_after_start_are_counted(self, tmp_path):
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            assert "0 file(s)" in topbar(app)
            assert "no files match" in notice(app)
            (tmp_path / "topic_freq.7.log").write_text(TEXT_BLOCK.format(ts=5, pid=7))
            await settle(pilot)
            assert "1 file(s)" in topbar(app)
            assert "1 windows" in topbar(app)

    async def test_unreadable_files_are_named_loudly(self, tmp_path):
        (tmp_path / "topic_freq.1.log").write_text("[INFO] something else entirely\n")
        app = PulseTopApp(str(tmp_path / "topic_freq.*.log"), poll_s=0.05)
        async with app.run_test(size=(140, 30)) as pilot:
            await settle(pilot)
            msg = notice(app)
            assert "topic_freq.1.log" in msg
            assert "not ros2_pulse probe output" in msg


class TestIssue59Theme:
    # Issue #59: outdoors the terminal is black on white, pulse-top forced black.
    def test_resolve_theme_flag_env_default(self, monkeypatch):
        from pulse_top.app import resolve_theme
        monkeypatch.delenv("PULSE_TOP_THEME", raising=False)
        assert resolve_theme(None) == "dark"
        monkeypatch.setenv("PULSE_TOP_THEME", "light")
        assert resolve_theme(None) == "light"
        assert resolve_theme("terminal") == "terminal"  # the flag wins over the env
        monkeypatch.setenv("PULSE_TOP_THEME", "bogus")
        with pytest.raises(ValueError):
            resolve_theme(None)

    @pytest.mark.parametrize("name", ["dark", "light", "terminal"])
    async def test_theme_applies(self, log, name):
        from pulse_top.app import PALETTES
        app = PulseTopApp(str(log), poll_s=0.05, theme=name)
        async with app.run_test(size=(140, 30)) as pilot:
            with open(log, "a") as f:
                f.write(window(100.0, [("/a", 5.0)]))
            await settle(pilot)
            assert app._pal is PALETTES[name]
            bg = app.screen.styles.background
            if name == "light":
                assert bg.brightness > 0.9
            elif name == "dark":
                assert bg.brightness < 0.1
            else:  # terminal: the terminal's own default background, never painted over
                assert bg.ansi == -1 or app.current_theme.background == "ansi_default"

    def test_light_theme_colours_are_dark_enough_for_white(self):
        # WCAG-ish: every text colour of the light palette must contrast with white.
        from textual.color import Color

        from pulse_top.app import PALETTES
        pal = PALETTES["light"]
        for c in (pal.accent, pal.good, pal.warn, pal.bad, pal.dim):
            assert Color.parse(c).brightness < 0.55, c
