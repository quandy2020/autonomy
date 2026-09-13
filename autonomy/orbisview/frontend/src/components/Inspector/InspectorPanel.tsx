import { useDataStore } from '@/store/dataStore';

export function InspectorPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const entries = Object.values(envelopes);
  return (
    <div className="panel log-panel">
      <h3>Message Inspector</h3>
      <pre>
        {entries.length === 0
          ? 'No envelopes yet — Connect and subscribe.'
          : entries
              .map((e) => {
                const flags = [
                  e.stale ? 'STALE' : null,
                  e.unsupported ? 'UNSUPPORTED' : null,
                ]
                  .filter(Boolean)
                  .join(' ');
                return (
                  `${e.channel}${flags ? ` [${flags}]` : ''}\n` +
                  `schema=${e.schema} seq=${e.sequence} enc=${e.encoding} ts=${e.timestamp}\n` +
                  JSON.stringify(e.payload ?? e.payload_b64, null, 2)
                );
              })
              .join('\n\n---\n\n')}
      </pre>
    </div>
  );
}
