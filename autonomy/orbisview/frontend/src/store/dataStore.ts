import { create } from 'zustand';
import type { ChannelInfo, StreamEnvelope } from '@/store/websocket/types';
import { STALE_THRESHOLD_MS } from '@/store/websocket/types';
import type { ConnState } from '@/store/websocket/client';

interface DataState {
  connectionState: ConnState;
  connected: boolean;
  channels: ChannelInfo[];
  /** Channels the UI has asked to subscribe (persists across reconnect). */
  subscribed: Record<string, number>;
  envelopes: Record<string, StreamEnvelope>;
  log: string[];
  setConnectionState: (v: ConnState) => void;
  setConnected: (v: boolean) => void;
  setChannels: (c: ChannelInfo[]) => void;
  markSubscribed: (channel: string, maxHz?: number) => void;
  markUnsubscribed: (channel: string) => void;
  clearSubscriptions: () => void;
  upsertEnvelope: (env: StreamEnvelope) => void;
  refreshStale: () => void;
  pushLog: (line: string) => void;
}

function isStale(env: StreamEnvelope, nowMs: number): boolean {
  if (!env.timestamp) return false;
  const ageMs = nowMs - env.timestamp / 1e6;
  return ageMs > STALE_THRESHOLD_MS;
}

/** Coalesce high-rate envelopes to one React store write per animation frame. */
const pendingEnvelopes = new Map<string, StreamEnvelope>();
let flushRaf: number | null = null;
let logTick = 0;

function flushPendingEnvelopes(
  apply: (envs: Record<string, StreamEnvelope>, logLine: string | null) => void,
): void {
  flushRaf = null;
  if (pendingEnvelopes.size === 0) return;
  const batch = new Map(pendingEnvelopes);
  pendingEnvelopes.clear();
  const now = Date.now();
  const merged: Record<string, StreamEnvelope> = {};
  let logLine: string | null = null;
  for (const [ch, env] of batch) {
    merged[ch] = { ...env, stale: isStale(env, now) || !!env.stale };
    // Log at most ~2 lines/sec worth of channels, never every image frame.
    if (++logTick % 8 === 0) {
      logLine = `${env.channel} seq=${env.sequence}${env.unsupported ? ' [unsupported]' : ''}`;
    }
  }
  apply(merged, logLine);
}

export const useDataStore = create<DataState>((set, get) => ({
  connectionState: 'offline',
  connected: false,
  channels: [],
  subscribed: {},
  envelopes: {},
  log: [],
  setConnectionState: (connectionState) =>
    set({ connectionState, connected: connectionState === 'online' }),
  setConnected: (connected) =>
    set({
      connected,
      connectionState: connected
        ? 'online'
        : get().connectionState === 'reconnecting'
          ? 'reconnecting'
          : 'offline',
    }),
  setChannels: (channels) => set({ channels }),
  markSubscribed: (channel, maxHz = 0) =>
    set((s) => ({ subscribed: { ...s.subscribed, [channel]: maxHz } })),
  markUnsubscribed: (channel) =>
    set((s) => {
      const next = { ...s.subscribed };
      delete next[channel];
      return { subscribed: next };
    }),
  clearSubscriptions: () => set({ subscribed: {} }),
  upsertEnvelope: (env) => {
    pendingEnvelopes.set(env.channel, env);
    if (flushRaf != null) return;
    flushRaf = window.requestAnimationFrame(() => {
      flushPendingEnvelopes((merged, logLine) => {
        set((s) => ({
          envelopes: { ...s.envelopes, ...merged },
          log: logLine ? [logLine, ...s.log].slice(0, 80) : s.log,
        }));
      });
    });
  },
  refreshStale: () => {
    const now = Date.now();
    set((s) => {
      let changed = false;
      const next: Record<string, StreamEnvelope> = {};
      for (const [k, env] of Object.entries(s.envelopes)) {
        const stale = isStale(env, now);
        if (stale !== !!env.stale) changed = true;
        next[k] = stale === !!env.stale ? env : { ...env, stale };
      }
      return changed ? { envelopes: next } : {};
    });
  },
  pushLog: (line) => set((s) => ({ log: [line, ...s.log].slice(0, 80) })),
}));
