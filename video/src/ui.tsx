import React from 'react';
import {interpolate, spring, useCurrentFrame, useVideoConfig} from 'remotion';
import {C, FONT} from './theme';

/** 0→1 with a soft spring, starting at `delay` frames. */
export const useRise = (delay = 0, damping = 200) => {
  const frame = useCurrentFrame();
  const {fps} = useVideoConfig();
  return spring({frame: frame - delay, fps, config: {damping}, durationInFrames: 24});
};

/** Fade+lift wrapper: appears at `delay`. */
export const Rise: React.FC<{delay?: number; children: React.ReactNode; style?: React.CSSProperties}> = ({
  delay = 0,
  children,
  style,
}) => {
  const v = useRise(delay);
  return (
    <div style={{opacity: v, transform: `translateY(${(1 - v) * 24}px)`, ...style}}>{children}</div>
  );
};

export const Eyebrow: React.FC<{children: React.ReactNode; color?: string}> = ({children, color = C.ink3}) => (
  <div style={{fontFamily: FONT.sans, fontSize: 26, letterSpacing: '0.14em', textTransform: 'uppercase', color, fontWeight: 600}}>
    {children}
  </div>
);

export const H1: React.FC<{children: React.ReactNode; size?: number; color?: string}> = ({children, size = 96, color = C.ink}) => (
  <div style={{fontFamily: FONT.display, fontSize: size, lineHeight: 1.05, fontWeight: 600, color, textWrap: 'balance' as never}}>
    {children}
  </div>
);

export const Mono: React.FC<{children: React.ReactNode; size?: number; color?: string; style?: React.CSSProperties}> = ({
  children,
  size = 34,
  color = C.ink,
  style,
}) => (
  <span style={{fontFamily: FONT.mono, fontSize: size, color, ...style}}>{children}</span>
);

/** Terminal card with traffic lights. */
export const Terminal: React.FC<{title: string; children: React.ReactNode; width?: number}> = ({title, children, width = 1500}) => (
  <div style={{width, borderRadius: 14, background: '#0a0f0d', border: `1px solid ${C.rule}`, boxShadow: '0 30px 80px rgba(0,0,0,.55)', overflow: 'hidden'}}>
    <div style={{height: 52, display: 'flex', alignItems: 'center', gap: 10, padding: '0 20px', background: C.surface, borderBottom: `1px solid ${C.rule}`}}>
      {['#f27d72', '#f2c96b', '#7ee2a8'].map((c) => (
        <div key={c} style={{width: 14, height: 14, borderRadius: 7, background: c}} />
      ))}
      <div style={{marginLeft: 'auto', marginRight: 'auto', fontFamily: FONT.mono, fontSize: 22, color: C.ink3}}>{title}</div>
    </div>
    <div style={{padding: '28px 34px', fontFamily: FONT.mono, fontSize: 30, lineHeight: 1.5, color: C.ink}}>{children}</div>
  </div>
);

/** Types `text` over `frames` frames starting at `delay`; returns the visible prefix. */
export const useTyped = (text: string, delay: number, frames: number) => {
  const frame = useCurrentFrame();
  const n = Math.round(interpolate(frame, [delay, delay + frames], [0, text.length], {extrapolateLeft: 'clamp', extrapolateRight: 'clamp'}));
  return text.slice(0, n);
};

export const Caret: React.FC = () => {
  const frame = useCurrentFrame();
  return <span style={{opacity: frame % 20 < 12 ? 1 : 0, color: C.accent}}>▍</span>;
};
