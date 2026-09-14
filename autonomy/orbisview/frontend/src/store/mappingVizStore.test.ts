import { beforeEach, describe, expect, it } from 'vitest';
import { useMappingVizStore } from './mappingVizStore';

beforeEach(() => {
  useMappingVizStore.setState({
    mode: 'demo',
    demo: {
      frameUrls: ['a.png', 'b.png', 'c.png'],
      frameIndex: 0,
      playing: false,
      intervalMs: 500,
      basemapMeta: null,
    },
    live: { mirrorMapToBasemap: false },
    elapsedMs: 0,
  });
});

describe('nextFrame', () => {
  it('advances until last then stops playing', () => {
    const s = useMappingVizStore.getState;
    expect(s().nextFrame()).toBe(true);
    expect(s().demo.frameIndex).toBe(1);
    expect(s().nextFrame()).toBe(true);
    expect(s().demo.frameIndex).toBe(2);
    expect(s().nextFrame()).toBe(false);
    expect(s().demo.playing).toBe(false);
    expect(s().demo.frameIndex).toBe(2);
  });
});

describe('reset', () => {
  it('returns to frame 0', () => {
    useMappingVizStore.getState().setFrameIndex(2);
    useMappingVizStore.getState().play();
    useMappingVizStore.getState().reset();
    expect(useMappingVizStore.getState().demo.frameIndex).toBe(0);
    expect(useMappingVizStore.getState().demo.playing).toBe(false);
  });
});

describe('play', () => {
  it('no-ops when no frames', () => {
    useMappingVizStore.setState({
      demo: {
        ...useMappingVizStore.getState().demo,
        frameUrls: [],
        playing: false,
      },
    });
    useMappingVizStore.getState().play();
    expect(useMappingVizStore.getState().demo.playing).toBe(false);
  });
});
