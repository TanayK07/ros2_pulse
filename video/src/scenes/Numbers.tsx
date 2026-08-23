import {AbsoluteFill} from 'remotion';
import {C, FONT} from '../theme';
import {Eyebrow, H1, Rise} from '../ui';
import {PROBE} from '../data';

const Tile: React.FC<{delay: number; big: string; unit: string; what: string; where: string; color?: string}> = ({delay, big, unit, what, where, color = C.accent}) => (
  <Rise delay={delay} style={{flex: 1}}>
    <div style={{background: C.surface, border: `1px solid ${C.rule}`, borderRadius: 12, padding: '30px 34px', height: 330, display: 'flex', flexDirection: 'column', gap: 8}}>
      <div style={{display: 'flex', alignItems: 'baseline', gap: 12}}>
        <span style={{fontFamily: FONT.display, fontSize: 96, fontWeight: 700, color, fontVariantNumeric: 'tabular-nums', lineHeight: 1}}>{big}</span>
        <span style={{fontFamily: FONT.sans, fontSize: 30, color: C.ink2}}>{unit}</span>
      </div>
      <div style={{fontFamily: FONT.sans, fontSize: 30, color: C.ink, marginTop: 10}}>{what}</div>
      <div style={{fontFamily: FONT.mono, fontSize: 22, color: C.ink3, marginTop: 'auto'}}>{where}</div>
    </div>
  </Rise>
);

export const Numbers: React.FC = () => (
  <AbsoluteFill style={{padding: '90px 120px', gap: 40}}>
    <Rise delay={0}>
      <Eyebrow>measured, not claimed</Eyebrow>
    </Rise>
    <Rise delay={6}>
      <H1 size={84}>Every number has a raw file behind it.</H1>
    </Rise>
    <div style={{display: 'flex', gap: 28, marginTop: 24}}>
      <Tile delay={20} big={String(PROBE.orinHotpathFixedNs)} unit="ns / message" what="counting hot path on Jetson AGX Orin" where="test/orin/out/hotpath/" color={C.good} />
      <Tile delay={30} big={`+${PROBE.orinJitterNs}`} unit="ns / message" what="opt-in jitter clock read on Orin (x86: +24)" where="test/orin/RESULTS.md" />
      <Tile delay={40} big={String(PROBE.sockets)} unit="sockets · subscribers" what="the probe never joins the graph" where="test/orin/out/report_on/" color={C.teal} />
      <Tile delay={50} big={PROBE.stressCpuPct} unit="% CPU" what="worst-case 4,900 msg/s stress, paired N=10 × 3 distros" where="bench/RESULTS.md" color={C.warn} />
    </div>
    <Rise delay={70} style={{marginTop: 'auto'}}>
      <span style={{fontFamily: FONT.sans, fontSize: 26, color: C.ink3}}>Humble · Jazzy · Kilted · stock binaries · CycloneDDS & FastDDS · x86-64 & aarch64</span>
    </Rise>
  </AbsoluteFill>
);
