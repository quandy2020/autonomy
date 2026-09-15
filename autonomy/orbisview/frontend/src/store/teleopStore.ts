import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export type TeleopGear = 'slow' | 'normal' | 'fast';

/** Scale applied on top of configured maxVx / maxWz. */
export const TELEOP_GEAR_SCALE: Record<TeleopGear, number> = {
  slow: 0.25,
  normal: 0.55,
  fast: 1,
};

export const TELEOP_GEAR_LABEL: Record<TeleopGear, string> = {
  slow: 'Slow',
  normal: 'Normal',
  fast: 'Fast',
};

export interface TeleopAck {
  vx: number;
  wz: number;
  at: number;
}

export interface TeleopState {
  gear: TeleopGear;
  /** Absolute caps at Fast gear (m/s, rad/s). */
  maxVx: number;
  maxWz: number;
  /** Stick deadzone 0–0.35. */
  deadzone: number;
  /** Settings drawer open. */
  settingsOpen: boolean;
  /** Armed — only then publish non-zero cmd_vel. */
  enabled: boolean;
  lastAck: TeleopAck | null;
  lastSendAt: number | null;
  setGear: (gear: TeleopGear) => void;
  setMaxVx: (v: number) => void;
  setMaxWz: (v: number) => void;
  setDeadzone: (v: number) => void;
  setSettingsOpen: (open: boolean) => void;
  setEnabled: (enabled: boolean) => void;
  /** Clear enable + timestamps (after stop / disconnect). */
  disarm: () => void;
  noteSend: () => void;
  noteAck: (vx: number, wz: number) => void;
  effectiveMax: () => { maxVx: number; maxWz: number };
}

export const useTeleopStore = create<TeleopState>()(
  persist(
    (set, get) => ({
      gear: 'slow',
      maxVx: 0.8,
      maxWz: 1.2,
      deadzone: 0.08,
      settingsOpen: false,
      enabled: false,
      lastAck: null,
      lastSendAt: null,
      setGear: (gear) => set({ gear }),
      setMaxVx: (maxVx) => set({ maxVx: clamp(maxVx, 0.05, 3) }),
      setMaxWz: (maxWz) => set({ maxWz: clamp(maxWz, 0.05, 4) }),
      setDeadzone: (deadzone) => set({ deadzone: clamp(deadzone, 0, 0.35) }),
      setSettingsOpen: (settingsOpen) => set({ settingsOpen }),
      setEnabled: (enabled) => set({ enabled }),
      disarm: () => set({ enabled: false }),
      noteSend: () => set({ lastSendAt: Date.now() }),
      noteAck: (vx, wz) => set({ lastAck: { vx, wz, at: Date.now() } }),
      effectiveMax: () => {
        const s = get();
        const scale = TELEOP_GEAR_SCALE[s.gear];
        return { maxVx: s.maxVx * scale, maxWz: s.maxWz * scale };
      },
    }),
    {
      name: 'orbisview-teleop-v1',
      partialize: (s) => ({
        gear: s.gear,
        maxVx: s.maxVx,
        maxWz: s.maxWz,
        deadzone: s.deadzone,
      }),
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<TeleopState>;
        return {
          ...current,
          gear: p.gear ?? 'slow',
          maxVx: p.maxVx ?? 0.8,
          maxWz: p.maxWz ?? 1.2,
          deadzone: p.deadzone ?? 0.08,
          settingsOpen: false,
          enabled: false,
          lastAck: null,
          lastSendAt: null,
        };
      },
    },
  ),
);

function clamp(v: number, lo: number, hi: number): number {
  if (!Number.isFinite(v)) return lo;
  return Math.min(hi, Math.max(lo, v));
}
