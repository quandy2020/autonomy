import { useEffect } from 'react';
import { asPayload, pickDisplayEnvelope } from '@/components/Channels/mapDisplayBinding';
import { useDataStore } from '@/store/dataStore';
import { useDisplayStore } from '@/store/displayStore';
import { useIndoorMapStore } from '@/store/indoorMapStore';
import { SCHEMAS } from '@/store/websocket/types';

/** Sync live FloorInfoArray / SemanticZoneArray envelopes into indoorMapStore. */
export function useIndoorMapLiveSync(): void {
  const envelopes = useDataStore((s) => s.envelopes);
  const displays = useDisplayStore((s) => s.displays);

  useEffect(() => {
    const floorEnv =
      pickDisplayEnvelope(envelopes, displays, 'floors') ??
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.FloorInfoArray);
    const zoneEnv =
      pickDisplayEnvelope(envelopes, displays, 'semantic') ??
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.SemanticZoneArray);

    if (floorEnv) {
      const payload = asPayload(floorEnv);
      if (payload) useIndoorMapStore.getState().applyLiveFloors(payload);
    }
    if (zoneEnv) {
      const payload = asPayload(zoneEnv);
      if (payload) useIndoorMapStore.getState().applyLiveZones(payload);
    }
  }, [envelopes, displays]);
}
