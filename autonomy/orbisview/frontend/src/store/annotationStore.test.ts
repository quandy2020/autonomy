import { beforeEach, describe, expect, it } from 'vitest';
import { useAnnotationStore } from './annotationStore';

beforeEach(() => {
  useAnnotationStore.setState({
    pois: [],
    shapes: [],
    draft: null,
    draftPreview: null,
    poiDefaultKind: 'charger',
    drawDefaultKind: 'polygon',
    selectedId: null,
  });
});

describe('commitDraft', () => {
  it('rejects short polygon', () => {
    const s = useAnnotationStore.getState;
    s().ensureDraft();
    s().appendDraftPoint(0, 0);
    s().appendDraftPoint(1, 0);
    expect(s().commitDraft()).toBe(false);
    expect(s().shapes).toHaveLength(0);
    expect(s().draft).not.toBeNull();
  });

  it('commits polygon with 3 points', () => {
    const s = useAnnotationStore.getState;
    s().ensureDraft();
    s().appendDraftPoint(0, 0);
    s().appendDraftPoint(1, 0);
    s().appendDraftPoint(1, 1);
    expect(s().commitDraft()).toBe(true);
    expect(s().shapes).toHaveLength(1);
    expect(s().draft).toBeNull();
  });
});

describe('importJson', () => {
  it('replaces on success and leaves state on failure', () => {
    useAnnotationStore.getState().addPoi({ x: 1, y: 1, kind: 'custom' });
    expect(() => useAnnotationStore.getState().importJson('{')).toThrow();
    expect(useAnnotationStore.getState().pois).toHaveLength(1);

    useAnnotationStore.getState().importJson(
      JSON.stringify({
        pois: [{ x: 2, y: 3, kind: 'elevator' }],
        shapes: [],
      }),
    );
    expect(useAnnotationStore.getState().pois).toHaveLength(1);
    expect(useAnnotationStore.getState().pois[0].x).toBe(2);
  });
});

describe('clearAll', () => {
  it('empties pois and shapes', () => {
    useAnnotationStore.getState().addPoi({ x: 0, y: 0, kind: 'charger' });
    useAnnotationStore.getState().clearAll();
    expect(useAnnotationStore.getState().pois).toHaveLength(0);
  });
});
