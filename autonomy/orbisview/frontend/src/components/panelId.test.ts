import { describe, expect, it } from 'vitest';
import { allocPanelInstanceId, panelBaseId } from './panelId';

describe('panelId', () => {
  it('panelBaseId strips instance suffix', () => {
    expect(panelBaseId('image')).toBe('image');
    expect(panelBaseId('image#2')).toBe('image');
    expect(panelBaseId('image#12')).toBe('image');
  });

  it('allocPanelInstanceId fills gaps', () => {
    expect(allocPanelInstanceId('image', [])).toBe('image');
    expect(allocPanelInstanceId('image', ['map2d'])).toBe('image');
    expect(allocPanelInstanceId('image', ['image'])).toBe('image#2');
    expect(allocPanelInstanceId('image', ['image', 'image#2'])).toBe('image#3');
    expect(allocPanelInstanceId('image', ['image', 'image#3'])).toBe('image#2');
  });
});
