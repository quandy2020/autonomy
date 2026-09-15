/**
 * Map navigation dispatch — single source for Go / Stop / Finish.
 *
 * Product rule: placing points only edits a draft route. Dispatch happens only
 * when the user presses Go (or Enter). 1 point → /goal_pose, 2+ → /goal_poses.
 * On Go: drop costmap-blocked waypoints, then send remaining in order.
 * On success/fail: clear draft waypoints so the Go button count resets to 0.
 */

import { useDataStore } from '@/store/dataStore';
import { useDisplayStore } from '@/store/displayStore';
import { useMapViewStore } from '@/store/mapViewStore';
import { useWaypointStore } from '@/store/waypointStore';
import { countReachedPrefix, filterFreeWaypoints } from '@/store/routeProgress';
import { wsClient } from '@/store/websocket/client';
import { SCHEMAS } from '@/store/websocket/types';
import {
  asPayload,
  pickDisplayEnvelope,
} from '@/components/Channels/mapDisplayBinding';
import type { OccupancyGridJson } from '@/renderer/map2d/types';

let lastGoAt = 0;
let sawHasGoal = false;
let sawActiveNavState = false;
let sawPathWhileActive = false;
let lastOutcomeAt = 0;

export type NavOutcome = 'succeeded' | 'failed' | 'canceled';

function activeCostOrMap(): OccupancyGridJson | null {
  const envelopes = useDataStore.getState().envelopes;
  const displays = useDisplayStore.getState().displays;
  return (
    asPayload<OccupancyGridJson>(
      pickDisplayEnvelope(envelopes, displays, 'costmap'),
    ) ??
    asPayload<OccupancyGridJson>(
      pickDisplayEnvelope(envelopes, displays, 'map'),
    )
  );
}

function resetWatch(): void {
  sawHasGoal = false;
  sawActiveNavState = false;
  sawPathWhileActive = false;
}

/** Clear draft waypoints after a terminal navigation outcome. */
export function finishNavigation(outcome: NavOutcome): void {
  const now = Date.now();
  // Ignore duplicate terminal signals (nav + path + proximity).
  if (now - lastOutcomeAt < 800) {
    useWaypointStore.getState().clear();
    useMapViewStore.getState().setRouteActive(false);
    return;
  }
  lastOutcomeAt = now;
  resetWatch();

  useWaypointStore.getState().clear();
  useMapViewStore.getState().setRouteActive(false);
  useMapViewStore.getState().setStatusMsg(
    outcome === 'succeeded'
      ? '导航成功 · 已清空航点'
      : outcome === 'failed'
        ? '导航失败 · 已清空航点'
        : '导航取消 · 已清空航点',
  );
}

export function goNavigation(): boolean {
  const connected = useDataStore.getState().connected;
  const store = useWaypointStore.getState();
  if (!connected || !store.waypoints.length) return false;

  // Debounce double-clicks / Enter+button (same goal spam breaks preempt).
  const now = Date.now();
  if (now - lastGoAt < 400) return false;
  lastGoAt = now;

  const grid = activeCostOrMap();
  const { kept, removed } = filterFreeWaypoints(store.waypoints, grid);
  if (removed > 0) {
    store.replaceAll(kept);
  }
  if (!kept.length) {
    useMapViewStore
      .getState()
      .setStatusMsg('无可走航点（均在障碍物内）');
    useMapViewStore.getState().setRouteActive(false);
    return false;
  }

  resetWatch();
  lastOutcomeAt = 0;

  if (kept.length === 1) {
    const wp = kept[0];
    wsClient.send({ op: 'set_goal', x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
    useMapViewStore.getState().setRouteActive(true);
    useMapViewStore.getState().setStatusMsg(
      removed > 0
        ? `出发 → 单点（已滤除 ${removed} 障碍点）`
        : `出发 → 单点 (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})`,
    );
    return true;
  }

  wsClient.send({
    op: 'set_route',
    waypoints: kept.map((wp) => ({
      x: wp.x,
      y: wp.y,
      yaw: wp.yaw ?? 0,
    })),
  });
  useMapViewStore.getState().setRouteActive(true);
  useMapViewStore.getState().setStatusMsg(
    removed > 0
      ? `出发 → ${kept.length} 点（已滤除 ${removed} 障碍点）`
      : `出发 → ${kept.length} 点路线`,
  );
  return true;
}

export function stopNavigation(clearDraft = true): void {
  resetWatch();
  if (clearDraft) {
    useWaypointStore.getState().clear();
  }
  useMapViewStore.getState().setRouteActive(false);
  // One cancel only — clear_route and clear_goal both map to /cancel_navigation.
  wsClient.send({ op: 'clear_goal' });
  useMapViewStore.getState().setStatusMsg(
    clearDraft ? '已停止并清空航点' : '已停止导航',
  );
}

type NavPayload = {
  state?: string;
  status?: string | number;
  has_goal?: boolean;
  distance_remaining?: number;
};

type PathPayload = { poses?: { x: number; y: number }[] };

/**
 * While a Go-dispatched route is active: prune reached points, and on
 * success/fail clear the draft so the toolbar count resets to 0.
 */
export function tickNavigationProgress(pose: {
  x: number;
  y: number;
} | null): void {
  const map = useMapViewStore.getState();
  if (!map.routeActive) {
    resetWatch();
    return;
  }

  const envelopes = useDataStore.getState().envelopes;
  const navEnv = Object.values(envelopes).find(
    (e) => e.schema === SCHEMAS.Navigation,
  );
  const nav = asPayload<NavPayload>(navEnv);
  const state = String(nav?.state ?? nav?.status ?? '').toUpperCase();

  if (nav?.has_goal === true) sawHasGoal = true;
  if (/FOLLOWING|NAVIGATING|PLANNING|ACTIVE|RUNNING/.test(state)) {
    sawActiveNavState = true;
  }

  if (/SUCCEEDED|SUCCESS/.test(state)) {
    finishNavigation('succeeded');
    return;
  }
  if (/FAILED|FAILURE/.test(state)) {
    finishNavigation('failed');
    return;
  }
  if (/CANCELED|CANCELLED/.test(state)) {
    finishNavigation('canceled');
    return;
  }
  // Mock / status drop: was following a goal, now idle without goal.
  if (
    sawHasGoal &&
    nav?.has_goal === false &&
    (!state || state === 'IDLE')
  ) {
    finishNavigation('succeeded');
    return;
  }
  if (sawActiveNavState && state === 'IDLE') {
    finishNavigation('succeeded');
    return;
  }

  const pathEnv = Object.values(envelopes).find(
    (e) => e.schema === SCHEMAS.Path2D,
  );
  const path = asPayload<PathPayload>(pathEnv);

  if (path?.poses && path.poses.length > 0) {
    sawPathWhileActive = true;
    if (pose) {
      const end = path.poses[path.poses.length - 1];
      const dx = pose.x - end.x;
      const dy = pose.y - end.y;
      if (dx * dx + dy * dy <= 0.55 * 0.55) {
        finishNavigation('succeeded');
        return;
      }
    }
  }

  // Path vanished after we had one, and still far from remaining goals → fail.
  if (
    sawPathWhileActive &&
    pose &&
    (!path?.poses || path.poses.length === 0)
  ) {
    const list = useWaypointStore.getState().waypoints;
    if (list.length > 0) {
      const last = list[list.length - 1];
      const dx = pose.x - last.x;
      const dy = pose.y - last.y;
      if (dx * dx + dy * dy > 1.0) {
        finishNavigation('failed');
        return;
      }
    }
  }

  if (!pose) return;

  const list = useWaypointStore.getState().waypoints;
  if (!list.length) {
    finishNavigation('succeeded');
    return;
  }

  const n = countReachedPrefix(list, pose.x, pose.y, 0.6);
  if (n > 0) {
    useWaypointStore.getState().removePrefix(n);
    const left = useWaypointStore.getState().waypoints.length;
    if (left === 0) {
      finishNavigation('succeeded');
    } else {
      useMapViewStore
        .getState()
        .setStatusMsg(`已过 ${n} 点 · 剩余 ${left}`);
    }
  }
}
