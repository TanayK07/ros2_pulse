import {AbsoluteFill, Img, interpolate, staticFile, useCurrentFrame, useVideoConfig} from 'remotion';
import {C, FONT} from '../theme';
import {Eyebrow, H1, Rise} from '../ui';
import meta from '../frames.json';

// Frames captured by video/capture_frames.py; frames.json labels the stall and Warns-tab
// frames from pixels, because the demo's window/frame offset drifts between captures.
const PRE = 3; // steady frames before the stall
const HOLD_STALL = 3; // slots per stall frame
const stallFrames = meta.stall.slice(0, 2);
const firstStall = meta.all.indexOf(stallFrames[0]);
const pre = meta.all.slice(Math.max(0, firstStall - PRE), firstStall);
const post = meta.all.slice(firstStall + stallFrames.length, firstStall + stallFrames.length + 9);
const SEQ = [...pre, ...stallFrames.flatMap((f) => Array(HOLD_STALL).fill(f) as string[]), ...post, ...meta.warns, meta.warns[meta.warns.length - 1]];
const FRAMES = SEQ.map((f) => `frames/${f}.png`);
const HOLD = 15; // frames per slot (0.5 s)
const STALL_AT = pre.length; // first stall slot
const STALL_SLOTS = stallFrames.length * HOLD_STALL;

export const Dashboard: React.FC = () => {
  const frame = useCurrentFrame();
  const {fps} = useVideoConfig();
  const idx = Math.min(FRAMES.length - 1, Math.floor(Math.max(0, frame - 20) / HOLD));
  // Slow push-in, then a harder zoom toward the /scan row while the stall is on screen.
  const push = interpolate(frame, [0, SCENES_TOTAL], [1, 1.06]);
  const stallT = interpolate(frame, [20 + STALL_AT * HOLD, 20 + (STALL_AT + 1) * HOLD, 20 + (STALL_AT + STALL_SLOTS) * HOLD, 20 + (STALL_AT + STALL_SLOTS + 1) * HOLD], [0, 1, 1, 0], {extrapolateLeft: 'clamp', extrapolateRight: 'clamp'});
  // Zoom about the upper-left of the frame, where the topic column and the first rows
  // sit, so /scan stays on screen as it grows.
  const zoom = push * (1 + 0.4 * stallT);
  return (
    <AbsoluteFill style={{padding: '70px 120px', gap: 24}}>
      <Rise delay={0}>
        <Eyebrow>pulse-top</Eyebrow>
      </Rise>
      <Rise delay={4}>
        <H1 size={72}>A dashboard that costs the robot nothing.</H1>
      </Rise>
      <Rise delay={12} style={{marginTop: 10, flex: 1, minHeight: 0}}>
        <div style={{height: 760, borderRadius: 14, overflow: 'hidden', border: `1px solid ${C.rule}`, boxShadow: '0 30px 80px rgba(0,0,0,.55)'}}>
          <Img
            src={staticFile(FRAMES[idx])}
            style={{width: '100%', display: 'block', transform: `scale(${zoom})`, transformOrigin: '0% 32%'}}
          />
        </div>
      </Rise>
      <div style={{position: 'absolute', right: 160, top: 700, opacity: stallT, transform: `translateY(${(1 - stallT) * 20}px)`}}>
        <div style={{background: C.bad, color: '#1a0a08', fontFamily: FONT.display, fontSize: 44, fontWeight: 700, padding: '14px 26px', borderRadius: 10, boxShadow: `0 10px 40px ${C.bad}66`}}>
          /scan stalled · gap &gt; 250 ms
        </div>
        <div style={{fontFamily: FONT.sans, fontSize: 26, color: C.ink2, marginTop: 12, textAlign: 'right'}}>a windowed mean would have passed it</div>
      </div>
      <div style={{position: 'absolute', bottom: 36, left: 120, fontFamily: FONT.sans, fontSize: 26, color: C.ink3}}>
        reads the probe's log · no ROS node · works over ssh · {fps} fps real-time capture
      </div>
    </AbsoluteFill>
  );
};

const SCENES_TOTAL = 13 * 30;
