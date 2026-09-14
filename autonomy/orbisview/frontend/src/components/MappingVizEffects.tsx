import { useMappingDemoPlayer } from '@/hooks/useMappingDemoPlayer';
import { useMappingLiveMirror } from '@/hooks/useMappingLiveMirror';

/** Mount once under Orbisview — demo timer + live mirror (idempotent via stores). */
export function MappingVizEffects() {
  useMappingDemoPlayer();
  useMappingLiveMirror();
  return null;
}
