import {AbsoluteFill} from 'remotion';
import {C, FONT} from '../theme';
import {H1, Mono, Rise} from '../ui';
import {LINKS} from '../data';

export const Cta: React.FC = () => (
  <AbsoluteFill style={{justifyContent: 'center', alignItems: 'center', gap: 40}}>
    <Rise delay={0} style={{textAlign: 'center'}}>
      <H1 size={120}>ros2_pulse</H1>
    </Rise>
    <Rise delay={8} style={{textAlign: 'center'}}>
      <span style={{fontFamily: FONT.sans, fontSize: 40, color: C.ink2}}>the heartbeat of your ROS 2 graph</span>
    </Rise>
    <Rise delay={20} style={{display: 'flex', flexDirection: 'column', gap: 18, alignItems: 'center', marginTop: 30}}>
      <Mono size={44} color={C.accent}>{LINKS.repo}</Mono>
      <Mono size={34} color={C.ink}>
        <span style={{color: C.ink3}}>$ </span>
        {LINKS.pip}
      </Mono>
    </Rise>
    <div style={{position: 'absolute', bottom: 48, fontFamily: FONT.sans, fontSize: 26, color: C.ink3}}>{LINKS.license} · star it if it saved you a morning</div>
  </AbsoluteFill>
);
