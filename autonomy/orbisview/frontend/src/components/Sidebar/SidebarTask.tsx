import { useMemo } from 'react';
import { Icon, IconLabel } from '@/components/icons';
import { useDataStore } from '@/store/dataStore';
import { useLayoutStore } from '@/store/layoutStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';

const TASKS: {
  id: string;
  panelId: string;
  title: string;
  icon: 'nav' | 'explore' | 'mapping' | 'waypoint' | 'layers';
  schema?: string;
  hint: string;
}[] = [
  {
    id: 'nav',
    panelId: 'navigation',
    title: 'Navigation',
    icon: 'nav',
    schema: SCHEMAS.Navigation,
    hint: 'Map 设目标 / 路线',
  },
  {
    id: 'explore',
    panelId: 'exploration',
    title: 'Exploration',
    icon: 'explore',
    schema: SCHEMAS.Exploration,
    hint: '自主探索任务状态',
  },
  {
    id: 'mapping',
    panelId: 'mapping',
    title: 'Mapping',
    icon: 'mapping',
    schema: SCHEMAS.Mapping,
    hint: '建图进度与质量',
  },
  {
    id: 'indoor_map',
    panelId: 'indoor_map',
    title: '室内地图',
    icon: 'layers',
    hint: '语义区 / 多楼层',
  },
  {
    id: 'annotations',
    panelId: 'annotations',
    title: 'POI管理',
    icon: 'waypoint',
    hint: 'POI / 绘制 · 导入导出',
  },
];

/** Compact task launcher for the left sidebar. */
export function SidebarTask() {
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const ensurePanel = useLayoutStore((s) => s.ensurePanel);

  const status = useMemo(() => {
    const out: Record<string, string> = {};
    for (const t of TASKS) {
      if (!t.schema) {
        out[t.id] = 'local';
        continue;
      }
      const e = Object.values(envelopes).find((x) => x.schema === t.schema);
      if (!e?.payload || typeof e.payload !== 'object') {
        out[t.id] = 'idle';
        continue;
      }
      const p = e.payload as Record<string, unknown>;
      const state = p.state ?? p.status ?? p.phase;
      out[t.id] = typeof state === 'string' ? state : e.stale ? 'stale' : 'live';
    }
    return out;
  }, [envelopes]);

  return (
    <div className="sidebar-section">
      <h3 className="sidebar-section-title">
        <IconLabel name="task" label="Task" size={13} />
      </h3>
      <p className="hint sidebar-hint">任务快捷入口 · 可打开对应面板</p>
      <div className="task-list">
        {TASKS.map((t) => (
          <div key={t.id} className="task-card">
            <div className="task-card-head">
              <Icon name={t.icon} size={14} />
              <strong>{t.title}</strong>
              <span className="task-status">{status[t.id]}</span>
            </div>
            <p className="hint">{t.hint}</p>
            <div className="task-card-actions">
              <button type="button" className="tab" onClick={() => ensurePanel(t.panelId)}>
                Open
              </button>
              {t.id === 'nav' ? (
                <button
                  type="button"
                  className="tab"
                  disabled={!connected}
                  onClick={() => wsClient.send({ op: 'clear_goal' })}
                >
                  Clear goal
                </button>
              ) : null}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
