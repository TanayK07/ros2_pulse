# Copyright 2026 ros2_pulse contributors
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""compare.py on synthetic probe logs and wire CSVs: python3 -m pytest examples/flood_compare"""

import csv
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare  # noqa: E402

T0 = 1_800_000_000  # whole unix seconds, so ts_ns stays exact


def pulse_record(end_s, window_s=1.0, cb=None, pub=None, partial=False, topic="/cmd_vel"):
    """One probe jsonl record shaped like formatWindowJsonl's output."""
    t = {"topic": topic}
    if pub is not None:
        t.update(pub_inter_hz=pub, pub_intra_hz=0.0)
    if cb is not None:
        t.update(recv_inter_hz=cb, recv_intra_hz=0.0, recv_endpoint_seen=True)
    rec = {"ts_ns": str(int(round(end_s * 1000)) * 1_000_000), "window_s": window_s}
    if partial:
        rec["partial"] = True
    rec.update(topics=[t] if len(t) > 1 else [], nodes=["/flood_controller"], warns=[],
               endpoints=[])
    return json.dumps(rec)


def write_lines(path, lines):
    path.write_text("".join(line + "\n" for line in lines))
    return str(path)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    return str(path)


@pytest.fixture
def controller_log(tmp_path):
    """10 windows ending T0+1 .. T0+10: first and last partial, 900 callbacks/s in between,
    except 450 in the window ending T0+6 (a stall)."""
    lines = []
    for k in range(1, 11):
        cb = 450.0 if k == 6 else 900.0
        lines.append(pulse_record(T0 + k, cb=cb, partial=k in (1, 10)))
    return write_lines(tmp_path / "pulse.jsonl", lines)


def rows_for(pulse_paths, wire_path=None, **wire_opts):
    run = compare.Run("t", list(pulse_paths), wire_path, {})
    opts = {"ts_at": "end", "window_s": None, "offset": 0.0}
    opts.update(wire_opts)
    return compare.compare_run(run, "/cmd_vel", opts, lambda m: None)


def by_end(rows):
    return {round(r.end - T0): r for r in rows}


# ---------------------------------------------------------------- wire shapes


def test_rate_rows_in_phase(tmp_path, controller_log):
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "topic", "msgs_per_s"],
                     [(T0 + k, "/cmd_vel", 1000.0) for k in range(0, 12)])
    rows = by_end(rows_for([controller_log], wire))
    assert rows[3].wire_hz == pytest.approx(1000.0)
    assert rows[3].cb_hz == pytest.approx(900.0)
    assert rows[6].cb_hz == pytest.approx(450.0)
    assert rows[3].pub_hz is None  # nothing publishes in the controller's process
    assert rows[3].full and not rows[1].full and "partial" in rows[10].flags


def test_rate_rows_half_a_window_out_of_phase(tmp_path, controller_log):
    # wire windows end at .5, so each probe window is half of two wire windows; the wire rate
    # steps from 1000 to 2000 with the row covering (T0+4.5, T0+5.5]
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "topic", "msgs_per_s"],
                     [(T0 + k + 0.5, "/cmd_vel", 1000.0 if k < 5 else 2000.0)
                      for k in range(0, 12)])
    rows = by_end(rows_for([controller_log], wire))
    assert rows[4].wire_hz == pytest.approx(1000.0)
    assert rows[5].wire_hz == pytest.approx(1500.0)  # (T0+4, T0+5]: half at 1000, half at 2000
    assert rows[6].wire_hz == pytest.approx(2000.0)


def test_count_rows_dds_names_and_millisecond_stamps(tmp_path, controller_log):
    wire = write_csv(tmp_path / "w.csv", ["Timestamp", "Topic", "Count"],
                     [((T0 + k) * 1000, "rt/cmd_vel", 500) for k in range(0, 12)]
                     + [((T0 + k) * 1000, "rt/other", 7) for k in range(0, 12)])
    rows = by_end(rows_for([controller_log], wire))
    assert rows[4].wire_hz == pytest.approx(500.0)


def test_rows_per_writer_add_up(tmp_path, controller_log):
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "topic", "msgs_per_s"],
                     [(T0 + k, "/cmd_vel", 300.0) for k in range(0, 12)] * 2)
    assert by_end(rows_for([controller_log], wire))[4].wire_hz == pytest.approx(600.0)


def test_window_column_and_start_stamps(tmp_path, controller_log):
    # 0.5 s rows stamped at their start
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "msgs_per_s", "window_s"],
                     [(T0 + k / 2, 800.0, 0.5) for k in range(0, 24)])
    rows = by_end(rows_for([controller_log], wire, ts_at="start"))
    assert rows[5].wire_hz == pytest.approx(800.0)


def test_one_row_per_message(tmp_path, controller_log):
    # 1000 events in each second, 500 in the sixth
    events = []
    for k in range(0, 12):
        n = 500 if k == 5 else 1000
        events += [(f"{T0 + k + (i + 0.5) / n:.6f}", "/cmd_vel") for i in range(n)]
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "topic"], events)
    rows = by_end(rows_for([controller_log], wire))
    assert rows[4].wire_hz == pytest.approx(1000.0)
    assert rows[6].wire_hz == pytest.approx(500.0)


def test_capture_that_misses_windows_is_not_a_rate(tmp_path, controller_log):
    # capture starts 0.6 s into the window ending T0+5
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "msgs_per_s"],
                     [(T0 + 4.6 + k, 1000.0) for k in range(1, 8)])
    rows = by_end(rows_for([controller_log], wire))
    assert rows[5].wire_hz is None and "no-wire" in rows[5].flags
    assert rows[7].wire_hz == pytest.approx(1000.0)


def test_capture_that_omits_silent_rows_reads_zero(tmp_path, controller_log):
    rows_in = [(T0 + k, 1000.0) for k in range(0, 12) if k != 7]
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "msgs_per_s"], rows_in)
    rows = by_end(rows_for([controller_log], wire))
    assert rows[7].wire_hz == pytest.approx(0.0) and rows[7].full


def test_wire_offset(tmp_path, controller_log):
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "msgs_per_s"],
                     [(T0 + k - 100, 1000.0 if k != 6 else 0.0) for k in range(0, 12)])
    rows = by_end(rows_for([controller_log], wire, offset=100.0))
    assert rows[6].wire_hz == pytest.approx(0.0)
    assert rows[5].wire_hz == pytest.approx(1000.0)


def test_wire_without_the_topic_is_an_error(tmp_path, controller_log):
    wire = write_csv(tmp_path / "w.csv", ["timestamp", "topic", "hz"], [(T0, "/scan", 10)])
    with pytest.raises(ValueError, match="no rows for /cmd_vel.*/scan"):
        rows_for([controller_log], wire)


# ---------------------------------------------------------------- probe side


def test_publisher_in_another_process_is_rebinned(tmp_path, controller_log):
    # publisher windows end at .25, 1000 Hz
    pub = write_lines(tmp_path / "pulse_pub.jsonl",
                      [pulse_record(T0 + k + 0.25, pub=1000.0) for k in range(0, 12)])
    rows = by_end(rows_for([controller_log, pub]))
    assert rows[4].pub_hz == pytest.approx(1000.0)
    assert rows[4].cb_hz == pytest.approx(900.0)
    assert rows[4].wire_hz is None


def test_partial_publisher_window_is_not_rebinned(tmp_path, controller_log):
    # the publisher's first window is diluted by its startup: it must not lower the
    # controller window it overlaps, that stretch is a gap instead
    lines = [pulse_record(T0 + 3.5, window_s=2.5, pub=100.0, partial=True)]
    lines += [pulse_record(T0 + k + 0.5, pub=1000.0) for k in range(4, 12)]
    pub = write_lines(tmp_path / "pulse_pub.jsonl", lines)
    rows = by_end(rows_for([controller_log, pub]))
    assert rows[4].pub_hz is None and "pub-gap" in rows[4].flags
    assert rows[5].pub_hz == pytest.approx(1000.0)


def test_publisher_window_without_the_topic_is_zero(tmp_path, controller_log):
    lines = [pulse_record(T0 + k, pub=None if k == 5 else 1000.0) for k in range(0, 12)]
    pub = write_lines(tmp_path / "pulse_pub.jsonl", lines)
    rows = by_end(rows_for([controller_log, pub]))
    assert rows[5].pub_hz == pytest.approx(0.0)


def test_text_format_log_is_refused(tmp_path):
    log = write_lines(tmp_path / "pulse.log", ["# ts_ns=1 window_s=1.000", "RECV /cmd_vel "
                                               "inter=1.0 intra=0.0", ""])
    assert compare.main([log]) == 2


def test_shared_file_warns(tmp_path):
    lines = []
    for k in range(1, 5):
        lines += [pulse_record(T0 + k, cb=10.0), pulse_record(T0 + k + 0.3, pub=10.0)]
    log = write_lines(tmp_path / "pulse.jsonl", lines)
    msgs = []
    compare.read_pulse(log, "/cmd_vel", msgs.append)
    assert any("several processes" in m for m in msgs)


def test_truncated_last_line_is_skipped(tmp_path, controller_log):
    with open(controller_log, "a") as f:
        f.write('{"ts_ns":"1800000011000000000","window_s":1.0,"top')
    msgs = []
    assert len(compare.read_pulse(controller_log, "/cmd_vel", msgs.append)) == 10
    assert msgs


# ---------------------------------------------------------------- end to end


def make_runs_dir(tmp_path):
    """Two run dirs as run.sh writes them, one wire CSV for the whole session."""
    out = tmp_path / "flood_out"
    wire_rows = []
    for i, (rel, depth) in enumerate((("best_effort", 1), ("reliable", 10))):
        d = out / f"{rel}_d{depth}"
        d.mkdir(parents=True)
        base = T0 + 100 * i
        cb = 400.0 if rel == "best_effort" else 1000.0
        write_lines(d / "pulse.jsonl", [pulse_record(base + k, cb=cb, partial=k in (1, 6))
                                        for k in range(1, 7)])
        (d / "meta.json").write_text(json.dumps({"reliability": rel, "depth": depth}))
        wire_rows += [(base + k, "rt/cmd_vel", 1000.0) for k in range(0, 8)]
    wire = write_csv(tmp_path / "wire.csv", ["timestamp", "topic", "msgs_per_s"], wire_rows)
    return str(out), wire


def test_session_dir_table_and_csv(tmp_path, capsys):
    out, wire = make_runs_dir(tmp_path)
    table = tmp_path / "table.csv"
    assert compare.main([out, wire, "--csv", str(table)]) == 0
    text = capsys.readouterr().out
    # reliable runs are listed first, whatever the directory order
    assert text.index("== reliable_d10") < text.index("== best_effort_d1")
    assert "4 full windows of 6" in text
    assert "callbacks/wire 40.00%" in text and "callbacks/wire 100.00%" in text
    with open(table) as f:
        got = list(csv.DictReader(f))
    assert len(got) == 12
    assert set(got[0]) == set(compare.CSV_FIELDS)
    be = [r for r in got if r["run"] == "best_effort_d1" and not r["flags"]]
    assert len(be) == 4
    assert all(float(r["cb_minus_wire_hz"]) == pytest.approx(-600.0) for r in be)
    assert all(r["reliability"] == "best_effort" and r["depth"] == "1" for r in be)


def test_max_diff_gate(tmp_path, capsys):
    out, wire = make_runs_dir(tmp_path)
    assert compare.main([os.path.join(out, "reliable_d10"), wire, "--max-diff-pct", "1"]) == 0
    assert compare.main([out, wire, "--max-diff-pct", "1", "--summary"]) == 1
    assert "== best_effort_d1" in capsys.readouterr().out


def test_gate_without_reference_fails(tmp_path, controller_log):
    assert compare.main([controller_log, "--max-diff-pct", "5"]) == 1


def test_loose_files_with_publisher_and_no_wire(tmp_path, controller_log, capsys):
    pub = write_lines(tmp_path / "pulse_pub.jsonl",
                      [pulse_record(T0 + k, pub=900.0) for k in range(0, 12)])
    assert compare.main([controller_log, pub, "--label", "dry", "--max-diff-pct", "60"]) == 0
    text = capsys.readouterr().out
    assert "== dry" in text and "callbacks/publishes" in text
