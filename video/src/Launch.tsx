import {AbsoluteFill} from 'remotion';
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

export const Launch: React.FC = () => (
  <AbsoluteFill style={{backgroundColor: C.bg}}>
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
