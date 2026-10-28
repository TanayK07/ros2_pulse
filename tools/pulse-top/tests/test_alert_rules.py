"""The example alerting rules (examples/prometheus) against the exporter's metric catalogue.

promtool tests the rules on hand-written series, so a metric or label renamed in export.py
leaves them green while the shipped rules stop matching anything. This reads the rules with
a regex (no YAML dependency) and checks each rule's names against _metrics().
"""

from __future__ import annotations

import pathlib
import re

import pytest

from pulse_top import export

# examples/ is not in the sdist; there this path does not exist and the test skips
RULES = pathlib.Path(__file__).parent.joinpath(
    "../../../examples/prometheus/ros2_pulse_alerts.yml").resolve()
TARGET = {"job", "instance"}  # added at scrape time, to every series
GROUPING = re.compile(r"\b(?:by|without|on|ignoring|group_left|group_right)\s*\(([^)]*)\)")


def rules(text):
    """(alert, expr part, annotations part) per rule. Comments and quoted strings are
    blanked in the expr part, so neither is read as a metric or a label name."""
    text = re.sub(r"(?m)^\s*#.*\n", "", text)
    for chunk in re.split(r"(?m)^\s*- (?:alert|record):", text)[1:]:
        alert, _, rest = chunk.partition("\n")
        expr, _, notes = rest.partition("annotations:")
        expr = re.sub(r"#.*", "", re.sub(r'"(?:[^"\\]|\\.)*"', '""', expr))
        yield alert.strip(), expr, notes


@pytest.mark.skipif(not RULES.exists(), reason="examples/ is only in the repository checkout")
def test_alert_rules_use_exported_metrics_and_labels():
    catalogue = {name: set(m.labels) for name, m in export._metrics().items()}
    seen = set()
    for alert, expr, notes in rules(RULES.read_text(encoding="utf-8")):
        metrics = set(re.findall(r"\bros2_pulse_\w+", expr))
        assert metrics <= set(catalogue), (alert, metrics - set(catalogue))
        seen |= metrics
        for name, matchers in re.findall(r"\b(ros2_pulse_\w+)\{([^}]*)\}", expr):
            used = set(re.findall(r"(\w+)\s*[!=]~?", matchers))
            assert used <= catalogue[name] | TARGET, (alert, name, used - catalogue[name])
        # by/without/on/ignoring/group_* and $labels.x can only name a label of a metric
        # this rule reads
        labels = TARGET.union(*(catalogue[m] for m in metrics))
        used = {lab.strip() for g in GROUPING.findall(expr) for lab in g.split(",")} - {""}
        used |= set(re.findall(r"\$labels\.(\w+)", notes))
        assert used <= labels, (alert, used - labels)
    assert seen, "no ros2_pulse_ metric found; did the rules file layout change?"
