"""Markdown summary of the probe's jsonl output: per-endpoint rates and node liveness."""
import glob
import json
import statistics
import sys
from collections import defaultdict

out = sys.argv[1]
print(f'## {sys.argv[2]}\n')
for path in sorted(glob.glob(f'{out}/topic_freq.*.log*')):
    wins = []
    for line in open(path):
        try:
            wins.append(json.loads(line))
        except ValueError:
            pass
    wins = wins[2:-1]  # drop attach ramp-up and the exit window
    if not wins:
        continue
    eps = defaultdict(lambda: {'hz': [], 'dt': 0.0})
    alive = defaultdict(int)
    warns = sum(len(w.get('warns', [])) for w in wins)
    for w in wins:
        for e in w.get('endpoints', []):
            k = (e.get('node') or '?', e['topic'], e['side'])
            eps[k]['hz'].append(e.get('inter_hz', 0) + e.get('intra_hz', 0))
            eps[k]['dt'] = max(eps[k]['dt'], e.get('max_dt_ms', 0))
        for n in w.get('nodes', []):
            alive[n] += 1
    print(f'### `{path.split("/")[-1]}`: {len(wins)} windows, {len(eps)} endpoints, '
          f'{len(alive)} nodes, {warns} warns\n')
    print('| node | live windows |\n|---|---|')
    for n, c in sorted(alive.items()):
        print(f'| `{n}` | {c}/{len(wins)} |')
    print('\n| node | topic | side | median hz | min hz | max gap ms |\n|---|---|---|---|---|---|')
    for (n, t, s), v in sorted(eps.items()):
        hz = v['hz'] + [0.0] * (len(wins) - len(v['hz']))
        if max(hz) == 0:
            continue
        print(f'| `{n}` | `{t}` | {s} | {statistics.median(hz):.2f} | {min(hz):.2f} | '
              f'{v["dt"]:.1f} |')
    print()
