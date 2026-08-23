import {AbsoluteFill, Audio, interpolate, staticFile} from 'remotion';
import {TransitionSeries, linearTiming} from '@remotion/transitions';
import {fade} from '@remotion/transitions/fade';
import {slide} from '@remotion/transitions/slide';
import {C} from './theme';
import {Hook} from './scenes/Hook';
import {Problem} from './scenes/Problem';
import {OneLine} from './scenes/OneLine';
import {Dashboard} from './scenes/Dashboard';
import {Numbers} from './scenes/Numbers';
import {Cta} from './scenes/Cta';

const FPS = 30;
const T = 15; // transition length, frames
export const SCENES = {
  hook: 4 * FPS,
  problem: 8 * FPS,
  oneLine: 7 * FPS,
  dashboard: 13 * FPS,
  numbers: 6 * FPS,
  cta: 4 * FPS,
};
const N_TRANSITIONS = 5;
export const LAUNCH_DURATION_FRAMES =
  Object.values(SCENES).reduce((a, b) => a + b, 0) - N_TRANSITIONS * T;

// Bed: "Digital Cobalt (Synthwave)" by AvigeiaAvetian, Pixabay Content License (no attribution,
// no redistribution of the file itself, so public/music/ is gitignored; see README).
const MUSIC_GAIN = 0.22;
const FADE_IN = 20;
const FADE_OUT = 75;

export const Launch: React.FC = () => (
  <AbsoluteFill style={{backgroundColor: C.bg}}>
    <Audio
      src={staticFile('music/digital-cobalt.mp3')}
      volume={(f) =>
        interpolate(f, [0, FADE_IN, LAUNCH_DURATION_FRAMES - FADE_OUT, LAUNCH_DURATION_FRAMES], [0, MUSIC_GAIN, MUSIC_GAIN, 0], {
          extrapolateLeft: 'clamp',
          extrapolateRight: 'clamp',
        })
      }
    />
    <TransitionSeries>
      <TransitionSeries.Sequence durationInFrames={SCENES.hook}>
        <Hook />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={linearTiming({durationInFrames: T})} />
      <TransitionSeries.Sequence durationInFrames={SCENES.problem}>
        <Problem />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition
        presentation={slide({direction: 'from-right'})}
        timing={linearTiming({durationInFrames: T})}
      />
      <TransitionSeries.Sequence durationInFrames={SCENES.oneLine}>
        <OneLine />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={linearTiming({durationInFrames: T})} />
      <TransitionSeries.Sequence durationInFrames={SCENES.dashboard}>
        <Dashboard />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={linearTiming({durationInFrames: T})} />
      <TransitionSeries.Sequence durationInFrames={SCENES.numbers}>
        <Numbers />
      </TransitionSeries.Sequence>
      <TransitionSeries.Transition presentation={fade()} timing={linearTiming({durationInFrames: T})} />
      <TransitionSeries.Sequence durationInFrames={SCENES.cta}>
        <Cta />
      </TransitionSeries.Sequence>
    </TransitionSeries>
  </AbsoluteFill>
);
