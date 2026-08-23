import {AbsoluteFill, interpolate, useCurrentFrame, useVideoConfig} from 'remotion';
import {C, FONT} from '../theme';
import {H1, Rise, Eyebrow} from '../ui';

/** A heartbeat: two beats per ~1.2 s, the way an ECG dot reads. */
const Pulse: React.FC = () => {
  const frame = useCurrentFrame();
  const {fps} = useVideoConfig();
  const t = (frame / fps) % 1.2;
  const beat = (at: number) => Math.exp(-Math.pow((t - at) / 0.06, 2));
  const s = 1 + 0.35 * (beat(0.15) + 0.7 * beat(0.38));
  const ring = interpolate(t, [0.15, 1.2], [0, 1], {extrapolateLeft: 'clamp'});
  return (
    <div style={{position: 'relative', width: 120, height: 120}}>
      <div style={{position: 'absolute', inset: 0, borderRadius: 60, border: `3px solid ${C.accent}`, opacity: 1 - ring, transform: `scale(${1 + ring * 1.6})`}} />
      <div style={{position: 'absolute', inset: 28, borderRadius: 40, background: C.accent, transform: `scale(${s})`, boxShadow: `0 0 60px ${C.accent}88`}} />
    </div>
  );
};

export const Hook: React.FC = () => (
  <AbsoluteFill style={{justifyContent: 'center', alignItems: 'center', gap: 60}}>
    <Rise delay={0}>
      <Pulse />
    </Rise>
    <Rise delay={10} style={{textAlign: 'center', maxWidth: 1400}}>
      <H1 size={104}>Is every topic flowing at the rate it should?</H1>
    </Rise>
    <Rise delay={30} style={{textAlign: 'center'}}>
      <Eyebrow color={C.ink2}>and which nodes are alive, right now, on the robot</Eyebrow>
    </Rise>
    <div style={{position: 'absolute', bottom: 48, fontFamily: FONT.mono, fontSize: 26, color: C.ink3}}>ros2_pulse</div>
  </AbsoluteFill>
);
