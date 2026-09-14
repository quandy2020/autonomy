import { useIndoorMapStore } from '@/store/indoorMapStore';

/** Compact floor switcher for Map2D. */
export function MapFloorBar() {
  const floors = useIndoorMapStore((s) => s.floors);
  const activeFloorId = useIndoorMapStore((s) => s.activeFloorId);
  const stepFloor = useIndoorMapStore((s) => s.stepFloor);

  if (!floors.length) return null;

  const active = floors.find((f) => f.id === activeFloorId) ?? floors[0];
  const idx = floors.findIndex((f) => f.id === active.id);

  return (
    <div className="map-floor-bar" role="toolbar" aria-label="楼层">
      <button
        type="button"
        className="map-floor-btn"
        disabled={idx <= 0}
        onClick={() => stepFloor(-1)}
        title="上一层"
      >
        ‹
      </button>
      <span className="map-floor-name" title={active.id}>
        {active.name}
      </span>
      <button
        type="button"
        className="map-floor-btn"
        disabled={idx >= floors.length - 1}
        onClick={() => stepFloor(1)}
        title="下一层"
      >
        ›
      </button>
    </div>
  );
}
