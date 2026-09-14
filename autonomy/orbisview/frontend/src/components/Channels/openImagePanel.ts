import { allocPanelInstanceId, panelBaseId } from '@/components/panelId';
import {
  collectMosaicIds,
  insertMosaicLeaf,
  useLayoutStore,
} from '@/store/layoutStore';
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
 * New panels tile into a balanced mosaic grid (row/column alternation).
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
  const imageCount = existing.filter((x) => panelBaseId(x) === 'image').length;
  // 1st extra → row (side-by-side); 2nd → column nest; then alternate.
  const direction: 'row' | 'column' = imageCount % 2 === 0 ? 'row' : 'column';
  layout.setMosaic(insertMosaicLeaf(cur, id, direction));
  usePanelOptsStore.getState().setImageChannel(id, channel ?? null);
  return id;
}
