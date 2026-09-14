import { allocPanelInstanceId } from '@/components/panelId';
import { collectMosaicIds, useLayoutStore } from '@/store/layoutStore';
import { usePanelOptsStore } from '@/store/panelOptsStore';

const IMAGE_TYPE_IDS = new Set([
  'sensor_msgs/Image',
  'sensor_msgs/CompressedImage',
]);

export function isImageDisplayType(typeId: string): boolean {
  return IMAGE_TYPE_IDS.has(typeId);
}

/**
 * Open (or focus) an Image mosaic panel for a channel.
 * Multiple Image panels are allowed; reuses one already bound to the same channel.
 */
export function openImagePanel(channel?: string | null): string {
  const layout = useLayoutStore.getState();
  const cur = layout.mosaic;
  const existing = collectMosaicIds(cur);
  const opts = usePanelOptsStore.getState().image;

  if (channel) {
    for (const [panelId, o] of Object.entries(opts)) {
      if (o.channel === channel && existing.includes(panelId)) {
        return panelId;
      }
    }
  }

  const id = allocPanelInstanceId('image', existing);
  if (!cur) {
    layout.setMosaic(id);
  } else {
    layout.setMosaic({
      type: 'split',
      direction: 'column',
      children: [cur, id],
      splitPercentages: [72, 28],
    });
  }
  usePanelOptsStore.getState().setImageChannel(id, channel ?? null);
  return id;
}
