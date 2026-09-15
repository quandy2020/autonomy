import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

function TaskPanel({ schema, title }: { schema: string; title: string }) {
  const envelopes = useDataStore((s) => s.envelopes);
  const payload = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === schema);
    return e?.payload ?? null;
  }, [envelopes, schema]);

  return (
    <div className="panel">
      <h3>{title}</h3>
      <pre>{payload ? JSON.stringify(payload, null, 2) : 'Waiting for data…'}</pre>
    </div>
  );
}

export function ExplorationPanel() {
  return <TaskPanel schema={SCHEMAS.Exploration} title="Exploration" />;
}

export function NavigationPanel() {
  const connected = useDataStore((s) => s.connected);
  const envelopes = useDataStore((s) => s.envelopes);
  const payload = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Navigation);
    return e?.payload ?? null;
  }, [envelopes]);

  return (
    <div className="panel">
      <h3>导航状态</h3>
      <p className="hint">在 Map 选「导航」加点 →「出发」。此处仅显示反馈。</p>
      <pre style={{ marginTop: 8 }}>
        {payload ? JSON.stringify(payload, null, 2) : connected ? '等待导航反馈…' : '未连接'}
      </pre>
    </div>
  );
}

export { MappingPanel } from './MappingPanel';
