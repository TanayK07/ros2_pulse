"""Capture pulse-top demo frames for the launch video, deterministically labelled.

Runs `pulse-top` headlessly against the demo writer (same source as the README GIF),
exports one Textual screenshot per demo window, rasterizes to 1920 px PNGs under
public/frames/, and writes src/frames.json with the indices of the frames that carry a
/scan stall (red row) and the Warns tab, detected from pixels, not assumed from timing,
because the demo's window/frame offset drifts by one between runs.

Usage (from tools/pulse-top, with its venv):
    env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/python ../../video/capture_frames.py
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

import cairosvg
from PIL import Image

HERE = Path(__file__).resolve().parent
FRAMES = HERE / "public" / "frames"
META = HERE / "src" / "frames.json"
WIDTH = 1920
N_TOPICS_FRAMES = 26
N_WARNS_FRAMES = 4
# cairosvg draws Fira Code wider than Textual's cell grid and runs collide; DejaVu sits on it.
FONT = os.environ.get("PULSE_TOP_SVG_FONT", "DejaVu Sans Mono")

sys.path.insert(0, str(HERE.parent / "tools" / "pulse-top"))
from textual.widgets import TabbedContent  # noqa: E402
from pulse_top.app import PulseTopApp  # noqa: E402
from pulse_top.demo import start_demo_writer  # noqa: E402

BAD = (0xF2, 0x7D, 0x72)  # pulse_top.app BAD, a stalled /scan row is painted with it


def has_color(png: Path, rgb: tuple[int, int, int], box: tuple[int, int, int, int], tol: int = 18) -> bool:
    raw = Image.open(png).convert("RGB").crop(box).tobytes()
    for i in range(0, len(raw), 3):
        if all(abs(raw[i + k] - rgb[k]) <= tol for k in range(3)):
            return True
    return False


async def main() -> None:
    FRAMES.mkdir(parents=True, exist_ok=True)
    for old in FRAMES.glob("f*.png"):
        old.unlink()
    real = start_demo_writer()
    link = "/tmp/robot.jsonl"  # short path so the topbar never wraps
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(real, link)

    app = PulseTopApp(link, poll_s=0.1)
    names: list[str] = []
    async with app.run_test(size=(132, 28)) as pilot:
        for _ in range(12):
            await pilot.pause(0.1)
        for i in range(N_TOPICS_FRAMES + N_WARNS_FRAMES):
            if i == N_TOPICS_FRAMES:
                app.query_one(TabbedContent).active = "warns"
                await pilot.pause(0.3)
            svg = app.export_screenshot().replace("Fira Code", FONT)
            name = f"f{i:02d}"
            cairosvg.svg2png(bytestring=svg.encode(), write_to=str(FRAMES / f"{name}.png"), output_width=WIDTH)
            names.append(name)
            for _ in range(10):
                await pilot.pause(0.1)

    # Topic column = left ~25 % of the table; the six demo rows sit at y 160-340 at 1920 px.
    stall = [n for n in names[:N_TOPICS_FRAMES] if has_color(FRAMES / f"{n}.png", BAD, (0, 160, 500, 340))]
    warns = names[N_TOPICS_FRAMES:]
    META.write_text(json.dumps({"all": names, "stall": stall, "warns": warns}, indent=1) + "\n")
    print(f"frames={len(names)} stall={stall} warns={warns}")
    if not stall:
        raise SystemExit("no stall frame detected, the demo loop did not reach windows 10-11 in the capture")


asyncio.run(main())
