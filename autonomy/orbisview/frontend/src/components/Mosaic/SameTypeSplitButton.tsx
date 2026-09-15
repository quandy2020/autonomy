import { useContext } from 'react';
import {
  MosaicContext,
  MosaicWindowContext,
  getNodeAtPath,
  getParentNode,
  isTabsNode,
  type MosaicDirection,
  type MosaicNode,
} from 'react-mosaic-component';

function useSplitSameType(createNode: () => string) {
  const { mosaicActions } = useContext(MosaicContext);
  const { mosaicWindowActions } = useContext(MosaicWindowContext);

  return (direction: MosaicDirection) => {
    const path = mosaicWindowActions.getPath();
    const root = mosaicActions.getRoot();
    if (!root) return;
    if (isTabsNode(getParentNode(root, path))) return;
    const currentNode = getNodeAtPath(root, path);
    if (currentNode == null) return;

    const second = createNode();
    if (!second || second === currentNode) return;

    const newSplit: MosaicNode<string> = {
      type: 'split',
      direction,
      children: [currentNode as MosaicNode<string>, second],
      splitPercentages: [50, 50],
    };
    mosaicActions.replaceWith(path, newSplit);
  };
}

/**
 * Explicit right / down split that clones the current panel type.
 * (Upstream SplitButton ignores window createNode and only auto-picks one axis.)
 */
export function SameTypeSplitButtons({ createNode }: { createNode: () => string }) {
  const split = useSplitSameType(createNode);
  return (
    <>
      <button
        type="button"
        title="向右拆分"
        aria-label="Split right"
        className="mosaic-default-control split-right-button"
        onClick={() => split('row')}
      />
      <button
        type="button"
        title="向下拆分"
        aria-label="Split down"
        className="mosaic-default-control split-down-button"
        onClick={() => split('column')}
      />
    </>
  );
}
