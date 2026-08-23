import {AbsoluteFill} from 'remotion';
import {C, FONT} from '../theme';
import {Eyebrow, H1, Mono, Rise} from '../ui';
import {OBSERVER} from '../data';

const Card: React.FC<{delay: number; cmd: string; lines: [string, string][]; accent: string}> = ({delay, cmd, lines, accent}) => (
  <Rise delay={delay} style={{flex: 1}}>
    <div style={{background: C.surface, border: `1px solid ${C.rule}`, borderTop: `4px solid ${accent}`, borderRadius: 12, padding: '28px 32px', minHeight: 470, display: 'flex', flexDirection: 'column', gap: 22}}>
      <Mono size={32} color={C.ink}>{cmd}</Mono>
      <div style={{display: 'flex', flexDirection: 'column', gap: 18, marginTop: 6}}>
        {lines.map(([big, small]) => (
          <div key={small} style={{display: 'flex', alignItems: 'baseline', gap: 16}}>
            <span style={{fontFamily: FONT.display, fontSize: 50, fontWeight: 700, color: accent, fontVariantNumeric: 'tabular-nums', whiteSpace: 'nowrap', flex: 'none'}}>{big}</span>
            <span style={{fontFamily: FONT.sans, fontSize: 25, color: C.ink2, lineHeight: 1.25}}>{small}</span>
          </div>
        ))}
      </div>
    </div>
  </Rise>
);

export const Problem: React.FC = () => (
  <AbsoluteFill style={{padding: '90px 120px', gap: 44}}>
    <Rise delay={0}>
      <Eyebrow>the tools you have</Eyebrow>
    </Rise>
    <Rise delay={6}>
      <H1 size={88}>Looking costs a core. Looking at intra-process changes what you see.</H1>
    </Rise>
    <div style={{display: 'flex', gap: 32, marginTop: 24}}>
      <Card
        delay={24}
        accent={C.warn}
        cmd="ros2 topic hz /points"
        lines={[
          [`${OBSERVER.hzWatcherCorePct} %`, 'of a core to watch one 100 KB topic'],
          [`${OBSERVER.hzReportedHz} Hz`, `printed, accurate; true rate ${OBSERVER.truePublishHz}`],
          ['blind', 'to intra-process delivery'],
        ]}
      />
      <Card
        delay={36}
        accent={C.bad}
        cmd="ros2 topic echo /points"
        lines={[
          [`${OBSERVER.echoWatcherCorePct} %`, 'of a core, one topic, to /dev/null'],
          ['1', 'topic at a time'],
          ['0', 'subscribers is what the probe adds instead'],
        ]}
      />
      <Card
        delay={48}
        accent={C.accent}
        cmd="hz on an intra-process topic"
        lines={[
          [`+${OBSERVER.intraCpuPct} %`, 'CPU on the process you are watching'],
          [`${OBSERVER.intraPubWindowsNone} → ${OBSERVER.intraPubWindowsWatched}`, 'windows serialized: the watcher switches a code path on'],
        ]}
      />
    </div>
    <Rise delay={70} style={{marginTop: 'auto'}}>
      <span style={{fontFamily: FONT.sans, fontSize: 26, color: C.ink3}}>measured · {OBSERVER.source} · ros:humble, paired trials, mean of N=10</span>
    </Rise>
  </AbsoluteFill>
);
