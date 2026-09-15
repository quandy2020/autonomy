import { wsClient } from '@/store/websocket/client';
import { useTeleopStore } from '@/store/teleopStore';

/** Zero cmd_vel and disarm teleop; safe when offline. */
export function emergencyStop(): void {
  useTeleopStore.getState().disarm();
  wsClient.cmdVel(0, 0);
  useTeleopStore.getState().noteSend();
}
