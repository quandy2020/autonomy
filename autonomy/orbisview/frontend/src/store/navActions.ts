/**
 * Map navigation dispatch — single source for Go / Stop.
 *
 * Product rule: placing points only edits a draft route. Dispatch happens only
 * when the user presses Go (or Enter). 1 point → /goal_pose, 2+ → /goal_poses.
 */

import { useDataStore } from '@/store/dataStore';
import { useMapViewStore } from '@/store/mapViewStore';
import { useWaypointStore } from '@/store/waypointStore';
import { wsClient } from '@/store/websocket/client';

let lastGoAt = 0;

export function goNavigation(): boolean {
  const connected = useDataStore.getState().connected;
  const waypoints = useWaypointStore.getState().waypoints;
  if (!connected || !waypoints.length) return false;

  // Debounce double-clicks / Enter+button (same goal spam breaks preempt).
  const now = Date.now();
  if (now - lastGoAt < 400) return false;
  lastGoAt = now;

  if (waypoints.length === 1) {
    const wp = waypoints[0];
    wsClient.send({ op: 'set_goal', x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
    useMapViewStore
      .getState()
      .setStatusMsg(`出发 → 单点 (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})`);
    return true;
  }

  wsClient.send({
    op: 'set_route',
    waypoints: waypoints.map((wp) => ({
      x: wp.x,
      y: wp.y,
      yaw: wp.yaw ?? 0,
    })),
  });
  useMapViewStore
    .getState()
    .setStatusMsg(`出发 → ${waypoints.length} 点路线`);
  return true;
}

export function stopNavigation(clearDraft = true): void {
  if (clearDraft) {
    useWaypointStore.getState().clear();
  }
  // One cancel only — clear_route and clear_goal both map to /cancel_navigation.
  wsClient.send({ op: 'clear_goal' });
  useMapViewStore.getState().setStatusMsg(
    clearDraft ? '已停止并清空航点' : '已停止导航',
  );
}
