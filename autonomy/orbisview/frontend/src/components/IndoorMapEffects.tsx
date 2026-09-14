import { useIndoorMapLiveSync } from '@/hooks/useIndoorMapLiveSync';

/** Mount once under Orbisview. */
export function IndoorMapEffects() {
  useIndoorMapLiveSync();
  return null;
}
