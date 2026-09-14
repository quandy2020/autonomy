import { describe, expect, it } from 'vitest';
import { basemapFromDemoFrame, sortMappingFrameUrls } from './mappingDemo';

describe('sortMappingFrameUrls', () => {
  it('sorts numeric suffixes naturally', () => {
    expect(
      sortMappingFrameUrls([
        'blob:x/stage_10.png',
        'blob:x/stage_2.png',
        'blob:x/stage_1.png',
      ]),
    ).toEqual([
      'blob:x/stage_1.png',
      'blob:x/stage_2.png',
      'blob:x/stage_10.png',
    ]);
  });
});

describe('basemapFromDemoFrame', () => {
  it('copies meta onto StaticSlamBasemap', () => {
    const b = basemapFromDemoFrame({
      imageSrc: 'blob:abc',
      meta: {
        originX: -10,
        originY: -10,
        resolution: 0.05,
        widthPx: 200,
        heightPx: 200,
      },
      label: 'demo',
    });
    expect(b.originX).toBe(-10);
    expect(b.widthPx).toBe(200);
    expect(b.imageSrc).toBe('blob:abc');
    expect(b.source).toBe('file');
  });
});
