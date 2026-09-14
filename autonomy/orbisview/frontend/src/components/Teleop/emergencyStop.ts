import { wsClient } from '@/store/websocket/client';

/** Zero cmd_vel; safe to call when offline. */
export function emergencyStop(): void {
  wsClient.cmdVel(0, 0);
}
