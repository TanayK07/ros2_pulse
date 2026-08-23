// Palette shared with pulse-top (tools/pulse-top/pulse_top/app.py) so the video and the
// product read as one thing.
import {loadFont as loadSans} from '@remotion/google-fonts/IBMPlexSans';
import {loadFont as loadMono} from '@remotion/google-fonts/IBMPlexMono';
import {loadFont as loadDisplay} from '@remotion/google-fonts/BarlowSemiCondensed';

const sans = loadSans('normal', {weights: ['400', '500', '600'], subsets: ['latin']});
const mono = loadMono('normal', {weights: ['400', '500'], subsets: ['latin']});
const display = loadDisplay('normal', {weights: ['600', '700'], subsets: ['latin']});

export const FONT = {
  sans: `${sans.fontFamily}, system-ui, sans-serif`,
  mono: `${mono.fontFamily}, ui-monospace, Menlo, monospace`,
  display: `${display.fontFamily}, 'Arial Narrow', sans-serif`,
};

export const C = {
  bg: '#0e1311',
  surface: '#141a17',
  ink: '#e4e9e5',
  ink2: '#aab4ad',
  ink3: '#76817a',
  rule: '#26302b',
  accent: '#b48cf2',
  good: '#7ee2a8',
  warn: '#f2c96b',
  bad: '#f27d72',
  teal: '#4fb8b0',
};
