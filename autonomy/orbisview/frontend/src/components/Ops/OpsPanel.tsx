import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';
import { IconLabel } from '@/components/icons';

/** Simulation / playback ops formerly in the bottom bar. */
export function OpsPanel() {
  const connectionState = useDataStore((s) => s.connectionState);
  const online = connectionState === 'online';

  return (
    <div className="panel ops-panel">
      <h3 style={{ marginTop: 0 }}>Ops</h3>
      <p className="muted ops-panel-hint">Dump · Clear · Playback</p>
      <div className="ops-actions">
        <button
          type="button"
          className="btn-icon"
          disabled={!online}
          onClick={() => wsClient.send({ op: 'dump_snapshot', path: '/tmp/orbisview_dump.json' })}
        >
          <IconLabel name="dump" label="Dump" size={13} />
        </button>
        <button
          type="button"
          className="btn-icon"
          disabled={!online}
          onClick={() => wsClient.send({ op: 'clear_sim' })}
        >
          <IconLabel name="clear" label="Clear" size={13} />
        </button>
        <button
          type="button"
          className="btn-icon"
          disabled={!online}
          onClick={() =>
            wsClient.send({
              op: 'playback_start',
              path: '/tmp/orbisview_record.jsonl',
              speed: 1,
            })
          }
        >
          <IconLabel name="play" label="Play" size={13} />
        </button>
        <button
          type="button"
          className="btn-icon"
          disabled={!online}
          onClick={() => wsClient.send({ op: 'playback_pause', paused: true })}
        >
          <IconLabel name="pause" label="Pause" size={13} />
        </button>
        <button
          type="button"
          className="btn-icon"
          disabled={!online}
          onClick={() => wsClient.send({ op: 'playback_stop' })}
        >
          <IconLabel name="stop" label="Stop" size={13} />
        </button>
      </div>
    </div>
  );
}
