import { Icon } from '@/components/icons';
import { useMapViewStore } from '@/store/mapViewStore';

/** Single toggle: shows the *other* view icon and switches on click. */
export function MapViewModeToggle() {
  const mode = useMapViewStore((s) => s.mode);
  const setMode = useMapViewStore((s) => s.setMode);
  const next = mode === '2d' ? '3d' : '2d';
  const icon = next === '3d' ? 'view3d' : 'map2d';
  const label = next === '3d' ? '切到 3D' : '切到 2D';

  return (
    <div className="map-rail-group" aria-label="Map view mode">
      <button
        type="button"
        title={label}
        aria-label={label}
        className="map-rail-btn"
        onClick={() => setMode(next)}
      >
        <Icon name={icon} size={16} />
        <span className="map-rail-btn-label">{next.toUpperCase()}</span>
      </button>
    </div>
  );
}
