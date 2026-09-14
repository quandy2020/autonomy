import { describe, expect, it } from 'vitest';
import {
  hitTestPoi,
  normalizeShape,
  parseAnnotationFixture,
  serializeAnnotations,
} from './annotations';

describe('parseAnnotationFixture', () => {
  it('parses valid pois and shapes', () => {
    const { pois, shapes } = parseAnnotationFixture(
      JSON.stringify({
        version: 1,
        pois: [{ id: 'p1', x: 1, y: 2, kind: 'charger', label: 'C1' }],
        shapes: [
          {
            id: 's1',
            kind: 'polygon',
            points: [
              [0, 0],
              [1, 0],
              [1, 1],
            ],
          },
        ],
      }),
    );
    expect(pois).toHaveLength(1);
    expect(pois[0].kind).toBe('charger');
    expect(shapes).toHaveLength(1);
  });

  it('skips short polygons', () => {
    const { shapes } = parseAnnotationFixture(
      JSON.stringify({
        shapes: [{ kind: 'polygon', points: [[0, 0], [1, 0]] }],
      }),
    );
    expect(shapes).toHaveLength(0);
  });
});

describe('normalizeShape', () => {
  it('accepts polyline with 2 points', () => {
    expect(
      normalizeShape({
        kind: 'polyline',
        points: [
          [0, 0],
          [1, 1],
        ],
      })?.points,
    ).toHaveLength(2);
  });
});

describe('hitTestPoi', () => {
  it('returns nearest within radius', () => {
    const hit = hitTestPoi(
      [
        { id: 'a', x: 0, y: 0, kind: 'custom' },
        { id: 'b', x: 1, y: 0, kind: 'custom' },
      ],
      0.1,
      0,
      0.5,
    );
    expect(hit?.id).toBe('a');
  });
});

describe('serializeAnnotations', () => {
  it('round-trips version 1', () => {
    const text = serializeAnnotations(
      [{ id: 'p', x: 0, y: 0, kind: 'elevator' }],
      [],
    );
    expect(JSON.parse(text).version).toBe(1);
    expect(parseAnnotationFixture(text).pois).toHaveLength(1);
  });
});
