import { useEffect, useState } from 'react';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';

type RecorderStatus = {
  op: 'recorder_status';
  recording: boolean;
  playing: boolean;
  paused?: boolean;
  recorded: number;
  play_index?: number;
  play_total?: number;
  speed?: number;
  loop?: boolean;
  record_path: string;
  play_path: string;
};

export function RecorderPanel() {
  const connected = useDataStore((s) => s.connected);
  const [path, setPath] = useState('/tmp/orbisview_record.jsonl');
  const [filter, setFilter] = useState('');
  const [speed, setSpeed] = useState(1);
  const [loop, setLoop] = useState(false);
  const [seekIndex, setSeekIndex] = useState(0);
  const [status, setStatus] = useState<RecorderStatus | null>(null);
  const [indexJson, setIndexJson] = useState('');

  useEffect(() => {
    if (!connected) return;
    const poll = window.setInterval(() => {
      wsClient.send({ op: 'recorder_status' });
    }, 1000);
    const off = wsClient.onMessage((msg) => {
      if ((msg as { op?: string }).op === 'recorder_status') {
        setStatus(msg as RecorderStatus);
      } else if ((msg as { op?: string }).op === 'bag_index') {
        setIndexJson(JSON.stringify(msg, null, 2));
      }
    });
    wsClient.send({ op: 'recorder_status' });
    return () => {
      window.clearInterval(poll);
      off();
    };
  }, [connected]);

  const channels = filter
    .split(',')
    .map((s) => s.trim())
    .filter(Boolean);

  const progress =
    status && status.play_total && status.play_total > 0
      ? Math.min(100, Math.round(((status.play_index ?? 0) / status.play_total) * 100))
      : 0;

  return (
    <div className="panel">
      <h3>Record / Playback</h3>
      <label className="row">
        Path
        <input value={path} onChange={(e) => setPath(e.target.value)} size={40} />
      </label>
      <label className="row">
        Channel filter (comma, empty=all)
        <input
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          size={40}
          placeholder="/orbisview/mock/pose,/orbisview/mock/path"
        />
      </label>
      <div className="row" style={{ gap: 8, marginTop: 8, flexWrap: 'wrap' }}>
        <button
          type="button"
          onClick={() =>
            wsClient.send({
              op: 'record_start',
              path,
              ...(channels.length ? { channels } : {}),
            })
          }
        >
          Record
        </button>
        <button type="button" onClick={() => wsClient.send({ op: 'record_stop' })}>
          Stop record
        </button>
        <button
          type="button"
          onClick={() => wsClient.send({ op: 'record_set_filter', channels })}
        >
          Apply filter
        </button>
      </div>
      <div className="row" style={{ gap: 8, marginTop: 8, flexWrap: 'wrap' }}>
        <label>
          Speed
          <input
            type="number"
            min={0.1}
            step={0.1}
            value={speed}
            onChange={(e) => setSpeed(Number(e.target.value) || 1)}
            style={{ width: 64, marginLeft: 4 }}
          />
        </label>
        <label>
          <input type="checkbox" checked={loop} onChange={(e) => setLoop(e.target.checked)} />
          Loop
        </label>
        <button
          type="button"
          onClick={() => wsClient.send({ op: 'playback_start', path, speed, loop })}
        >
          Play
        </button>
        <button
          type="button"
          onClick={() => wsClient.send({ op: 'playback_pause', paused: true })}
        >
          Pause
        </button>
        <button
          type="button"
          onClick={() => wsClient.send({ op: 'playback_pause', paused: false })}
        >
          Resume
        </button>
        <button type="button" onClick={() => wsClient.send({ op: 'playback_stop' })}>
          Stop play
        </button>
      </div>
      <div className="row" style={{ gap: 8, marginTop: 8, flexWrap: 'wrap' }}>
        <label>
          Seek index
          <input
            type="number"
            min={0}
            value={seekIndex}
            onChange={(e) => setSeekIndex(Number(e.target.value) || 0)}
            style={{ width: 80, marginLeft: 4 }}
          />
        </label>
        <button
          type="button"
          onClick={() => wsClient.send({ op: 'playback_seek', index: seekIndex })}
        >
          Seek
        </button>
        <button type="button" onClick={() => wsClient.send({ op: 'bag_index', path })}>
          Index
        </button>
        <button type="button" onClick={() => wsClient.send({ op: 'recorder_status' })}>
          Status
        </button>
      </div>
      {status && (
        <div style={{ marginTop: 12 }}>
          <div className="row" style={{ gap: 12 }}>
            <span>{status.recording ? '● REC' : '○ idle'}</span>
            <span>
              {status.playing ? (status.paused ? '❚❚ paused' : '▶ playing') : '■ stopped'}
            </span>
            <span>
              frames {status.play_index ?? 0}/{status.play_total ?? 0}
            </span>
            <span>recorded={status.recorded}</span>
          </div>
          <div
            style={{
              marginTop: 6,
              height: 8,
              background: '#333',
              borderRadius: 4,
              overflow: 'hidden',
            }}
          >
            <div
              style={{
                width: `${progress}%`,
                height: '100%',
                background: '#4caf50',
              }}
            />
          </div>
        </div>
      )}
      <pre style={{ marginTop: 12 }}>{status ? JSON.stringify(status, null, 2) : '—'}</pre>
      {indexJson ? <pre style={{ marginTop: 8 }}>{indexJson}</pre> : null}
    </div>
  );
}
