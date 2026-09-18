#!/usr/bin/env python3
"""Figures for the observer-effect write-up (docs/blog/2026-09-18-watching-a-ros2-topic-changes-it.md).

Inputs, all in this repository:
  bench/out/observer_effect/observer_effect.csv   per-trial rows written by bench/run_observer_effect.sh
                                                  (N=10 trials x 4 rotated arms; 4 consecutive rows = 1 trial)
  bench/RESULTS.md, "End-to-end overhead" table   the probe's pooled whole-process cost, +1.9 % +/- 0.7 % SEM
                                                  (PROBE_PCT / PROBE_SEM below; it comes from a different harness,
                                                  run_overhead_repeated.sh, so it is not derivable from the CSV)
Constants taken from the harness: the watcher is attached for WATCH = DUR - 2 = 8 s
(run_observer_effect.sh), so a watcher's CPU seconds / 8 s is its share of one core, which is
how bench/RESULTS.md gets "0.566 s = 7.1 % of a core".

Outputs (1200 px wide, light background):
  docs/assets/blog/cli-cost-per-topic.png
  docs/assets/blog/intra-observer-effect.png

Run from the repository root:  python3 bench/plot_observer_effect.py
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
CSV = ROOT / "bench" / "out" / "observer_effect" / "observer_effect.csv"
OUT = ROOT / "docs" / "assets" / "blog"

WATCH_S = 8.0  # seconds the watcher was attached per run (run_observer_effect.sh: WATCH=$((DUR - 2)), DUR=10)
PROBE_PCT, PROBE_SEM = 1.9, 0.7  # bench/RESULTS.md, pooled paired diff as % of baseline

# Colours: categorical slots 1 and 2 of the reference palette, text in ink, grid recessive.
BLUE, ORANGE, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e6e6e3"


def mean_sem(xs: list[float]) -> tuple[float, float]:
    n = len(xs)
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / (n - 1) if n > 1 else 0.0
    return m, math.sqrt(var) / math.sqrt(n)


def load() -> dict[str, list[dict]]:
    """Group rows by arm; attach a 1-based trial index (4 consecutive rows form one trial)."""
    arms: dict[str, list[dict]] = {}
    with CSV.open() as f:
        for i, row in enumerate(csv.DictReader(f)):
            rec = {k: (float(v) if k != "arm" else v) for k, v in row.items()}
            rec["trial"] = i // 4 + 1
            arms.setdefault(row["arm"], []).append(rec)
    return arms


def style(ax) -> None:
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=11)
    ax.yaxis.grid(True, color=GRID, linewidth=1)
    ax.set_axisbelow(True)


def fig_cli_cost(arms: dict) -> None:
    hz_m, hz_s = mean_sem([r["watcher_cpu_s"] for r in arms["hz"]])
    echo_m, echo_s = mean_sem([r["watcher_cpu_s"] for r in arms["echo"]])
    labels = [
        "ros2 topic hz\n(one 100 KB topic at 50 Hz)",
        "ros2 topic echo > /dev/null\n(same topic)",
        "ros2_pulse probe\n(whole process, 91 endpoints,\n~4,900 msg/s)",
    ]
    vals = [hz_m / WATCH_S * 100, echo_m / WATCH_S * 100, PROBE_PCT]
    errs = [hz_s / WATCH_S * 100, echo_s / WATCH_S * 100, PROBE_SEM]
    colors = [BLUE, BLUE, ORANGE]

    fig, ax = plt.subplots(figsize=(12, 6), dpi=100)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    bars = ax.bar(labels, vals, yerr=errs, width=0.5, color=colors, capsize=4,
                  error_kw={"ecolor": INK, "elinewidth": 1})
    for b, v, e in zip(bars, vals, errs):
        ax.text(b.get_x() + b.get_width() / 2, v + e + 0.8, f"{v:.1f} %  (SEM {e:.1f})",
                ha="center", va="bottom", fontsize=11, color=INK)
    ax.set_ylabel("CPU, % of one core", color=MUTED, fontsize=11)
    ax.set_ylim(0, 36)
    ax.set_title("What it costs to watch a topic", loc="left", fontsize=14, color=INK, pad=22)
    ax.text(0, 1.01, "Left two: the CLI tool's own CPU while attached, per watched topic. "
                     "Right: the probe's added CPU across the whole workload (paired trials).",
            transform=ax.transAxes, fontsize=9.5, color=MUTED, va="bottom")
    style(ax)
    fig.text(0.01, 0.01, "ros:humble, N=10, mean +/- SEM. Source: bench/out/observer_effect/observer_effect.csv "
                         "(watcher CPU s / 8 s attached) and bench/RESULTS.md (probe, pooled).",
             fontsize=8.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(OUT / "cli-cost-per-topic.png")
    plt.close(fig)


def fig_intra_effect(arms: dict) -> None:
    none = sorted(arms["none"], key=lambda r: r["trial"])
    intra = sorted(arms["intra"], key=lambda r: r["trial"])
    assert [r["trial"] for r in none] == [r["trial"] for r in intra]
    trials = [int(r["trial"]) for r in none]
    a = [r["intra_cpu_s"] for r in none]
    b = [r["intra_cpu_s"] for r in intra]
    a_m, a_s = mean_sem(a)
    b_m, b_s = mean_sem(b)
    pct = (b_m / a_m - 1) * 100

    fig, ax = plt.subplots(figsize=(12, 6), dpi=100)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    w = 0.38
    xs = list(range(len(trials)))
    ax.bar([x - w / 2 for x in xs], a, width=w - 0.02, color=BLUE,
           label=f"nothing watching: mean {a_m:.3f} s (SEM {a_s:.3f})")
    ax.bar([x + w / 2 for x in xs], b, width=w - 0.02, color=ORANGE,
           label=f"ros2 topic hz /intra_0 attached: mean {b_m:.3f} s (SEM {b_s:.3f}), +{pct:.0f} %")
    ax.set_xticks(xs)
    ax.set_xticklabels([f"trial {t}" for t in trials])
    ax.set_ylabel("intra-process farm CPU, seconds per 10 s run", color=MUTED, fontsize=11)
    ax.set_ylim(0, 0.6)
    # dashed lines mark the two arm means; the legend carries their values
    ax.axhline(a_m, color=BLUE, linewidth=1, linestyle=(0, (4, 3)))
    ax.axhline(b_m, color=ORANGE, linewidth=1, linestyle=(0, (4, 3)))
    ax.set_title("The intra-process observer effect: the watched process's CPU, with and without hz",
                 loc="left", fontsize=14, color=INK, pad=22)
    ax.text(0, 1.01, "15 intra-process topics at 100 Hz in one process. Attaching hz to one of them makes the "
                     "publisher serialize; higher in all 10 trials.",
            transform=ax.transAxes, fontsize=9.5, color=MUTED, va="bottom")
    ax.legend(frameon=False, loc="upper left", fontsize=10)
    style(ax)
    fig.text(0.01, 0.01, "ros:humble, CycloneDDS, N=10 rotated arms. Source: bench/out/observer_effect/observer_effect.csv, "
                         "column intra_cpu_s, arms none and intra.",
             fontsize=8.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(OUT / "intra-observer-effect.png")
    plt.close(fig)
    print(f"intra farm CPU: none {a_m:.3f} +/- {a_s:.3f}, hz attached {b_m:.3f} +/- {b_s:.3f}, +{pct:.1f} %")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    arms = load()
    for arm in ("none", "hz", "echo", "intra"):
        assert len(arms[arm]) == 10, (arm, len(arms[arm]))
    fig_cli_cost(arms)
    fig_intra_effect(arms)
    hz_m, _ = mean_sem([r["watcher_cpu_s"] for r in arms["hz"]])
    echo_m, _ = mean_sem([r["watcher_cpu_s"] for r in arms["echo"]])
    print(f"watcher CPU: hz {hz_m:.3f} s = {hz_m / WATCH_S * 100:.1f} % of a core, "
          f"echo {echo_m:.3f} s = {echo_m / WATCH_S * 100:.1f} %")
    print(f"wrote {OUT / 'cli-cost-per-topic.png'} and {OUT / 'intra-observer-effect.png'}")


if __name__ == "__main__":
    main()
