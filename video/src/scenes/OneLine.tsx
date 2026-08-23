import {AbsoluteFill, useCurrentFrame} from 'remotion';
import {C} from '../theme';
import {Caret, Eyebrow, H1, Rise, Terminal, useTyped} from '../ui';

const CMD = 'export LD_PRELOAD=libros2_pulse.so';
const LOG: [string, string][] = [
  ['# ts_ns=1782887153899445923 window_s=5.000', C.ink3],
  ['TOPIC /scan 20.000000', C.ink],
  ['RECV  /scan inter=20.000000 intra=0.000000', C.ink],
  ['RECV  /points inter=0.000000 intra=30.000000', C.good],
  ['JITTER /scan recv max_dt_ms=21.284', C.ink],
  ['NODE  /perception', C.ink2],
  ['NODE  /planner', C.ink2],
];

export const OneLine: React.FC = () => {
  const frame = useCurrentFrame();
  const typed = useTyped(CMD, 12, 40);
  const logStart = 66;
  return (
    <AbsoluteFill style={{padding: '90px 120px', gap: 36}}>
      <Rise delay={0}>
        <Eyebrow>the probe</Eyebrow>
      </Rise>
      <Rise delay={4}>
        <H1 size={80}>One line. No subscriber. No rebuild. No privileges.</H1>
      </Rise>
      <Rise delay={10} style={{marginTop: 20}}>
        <Terminal title="robot · bash">
          <div>
            <span style={{color: C.accent}}>$ </span>
            {typed}
            {frame < logStart ? <Caret /> : null}
          </div>
          <div style={{color: C.ink3}}>$ ros2 launch your_stack your.launch.py</div>
          <div style={{color: C.ink3}}>$ tail -f /tmp/pulse.log</div>
          {LOG.map(([line, color], i) => {
            const at = logStart + i * 8;
            const on = frame >= at;
            return (
              <div key={line} style={{color, opacity: on ? 1 : 0, fontWeight: color === C.good ? 600 : 400}}>
                {line}
                {color === C.good && on ? <span style={{color: C.good, marginLeft: 28}}>← intra-process: invisible to every other tool</span> : null}
              </div>
            );
          })}
        </Terminal>
      </Rise>
    </AbsoluteFill>
  );
};
