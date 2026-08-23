# Launch video

A 40-second silent-with-captions launch video for `ros2_pulse`, generated with
[Remotion](https://www.remotion.dev/) so every number on screen is a value in
[`src/data.ts`](src/data.ts) that cites the committed file it came from, so the video can be
re-rendered when the numbers move, and never drift from them.

Not part of the ROS package, not built in CI. Rendered output is attached to the GitHub
Release, not committed.

## Scenes

| # | Scene | What it shows | Numbers from |
|---|---|---|---|
| 1 | Hook | the question: is every topic flowing at the rate it should? | — |
| 2 | The tools you have | `hz` / `echo` cost per watched topic; the intra-process observer effect | `bench/RESULTS.md` (observer effect) |
| 3 | The probe | one `LD_PRELOAD` line, the log it writes, the intra-process line | README "What you get" |
| 4 | pulse-top | real frames of `pulse-top --demo` through a `/scan` stall, then the Warns tab | `capture_frames.py` |
| 5 | Measured, not claimed | hot path, jitter clock read, sockets, stress CPU | `test/orin/RESULTS.md`, `bench/RESULTS.md` |
| 6 | CTA | repo, `pip install ros2-pulse-top`, license | — |

## Build

```bash
# 1. dashboard frames (needs tools/pulse-top's venv: textual, cairosvg, pillow)
cd tools/pulse-top
env -u PYTHONPATH -u AMENT_PREFIX_PATH .venv/bin/python ../../video/capture_frames.py
#    -> video/public/frames/f*.png and video/src/frames.json (stall / warns frames found by pixel colour)

# 2. render
cd ../../video
npm install
npm run render            # out/ros2_pulse-launch.mp4, 1920x1080, 30 fps, h264 crf 18
npm run dev               # Remotion Studio, to scrub and tweak
```

Frames are rasterized from Textual SVG screenshots with cairosvg. cairosvg draws Fira Code
wider than Textual's cell grid (text runs collide), so the SVG font is swapped for DejaVu Sans
Mono, which sits on the grid (`PULSE_TOP_SVG_FONT` to override).

## Music

Bed track: *Digital Cobalt (Synthwave)* by AvigeiaAvetian
(https://pixabay.com/music/synthwave-digital-cobalt-synthwave-580426/), Pixabay Content License
(free for commercial use, no attribution required). The license does not allow redistributing
the file on its own, so `public/music/` is gitignored: download the MP3 from that page and save
it as `public/music/digital-cobalt.mp3` before rendering. Gain 0.22, 20-frame fade in, 75-frame
fade out (`src/Launch.tsx`).

## Updating a number

Change it in `src/data.ts` (with the file it now comes from), re-render. Nothing else in the
scenes carries a literal measurement.
