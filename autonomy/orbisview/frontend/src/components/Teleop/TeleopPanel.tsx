import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';
import {
  TELEOP_GEAR_LABEL,
  type TeleopGear,
  useTeleopStore,
} from '@/store/teleopStore';
import { envelopeJsonPayload, readNumericPath } from '@/components/Charts/fieldPaths';
import { emergencyStop } from './emergencyStop';

const SEND_HZ = 15;
const CMD_CHANNEL = '/cmd_vel';

type Keys = {
  w: boolean;
  a: boolean;
  s: boolean;
  d: boolean;
  up: boolean;
  down: boolean;
  left: boolean;
  right: boolean;
};

const EMPTY_KEYS: Keys = {
  w: false,
  a: false,
  s: false,
  d: false,
  up: false,
  down: false,
  left: false,
  right: false,
};

function applyDeadzone(v: number, dz: number): number {
  if (Math.abs(v) < dz) return 0;
  const sign = v < 0 ? -1 : 1;
  return sign * (Math.abs(v) - dz) / Math.max(1e-6, 1 - dz);
}

/** Mosaic teleop: enable-gated stick + WASD, gear limits, gauges, STOP. */
export function TeleopPanel() {
  const connected = useDataStore((s) => s.connected);
  const connectionState = useDataStore((s) => s.connectionState);
  const envelopes = useDataStore((s) => s.envelopes);

  const enabled = useTeleopStore((s) => s.enabled);
  const gear = useTeleopStore((s) => s.gear);
  const maxVxCfg = useTeleopStore((s) => s.maxVx);
  const maxWzCfg = useTeleopStore((s) => s.maxWz);
  const deadzone = useTeleopStore((s) => s.deadzone);
  const settingsOpen = useTeleopStore((s) => s.settingsOpen);
  const lastAck = useTeleopStore((s) => s.lastAck);
  const lastSendAt = useTeleopStore((s) => s.lastSendAt);
  const setGear = useTeleopStore((s) => s.setGear);
  const setMaxVx = useTeleopStore((s) => s.setMaxVx);
  const setMaxWz = useTeleopStore((s) => s.setMaxWz);
  const setDeadzone = useTeleopStore((s) => s.setDeadzone);
  const setSettingsOpen = useTeleopStore((s) => s.setSettingsOpen);
  const setEnabled = useTeleopStore((s) => s.setEnabled);
  const noteSend = useTeleopStore((s) => s.noteSend);
  const noteAck = useTeleopStore((s) => s.noteAck);

  const maxVx = maxVxCfg * (gear === 'slow' ? 0.25 : gear === 'normal' ? 0.55 : 1);
  const maxWz = maxWzCfg * (gear === 'slow' ? 0.25 : gear === 'normal' ? 0.55 : 1);

  const [stick, setStick] = useState({ x: 0, y: 0 });
  const [held, setHeld] = useState({ vx: 0, wz: 0 });
  const [keysLit, setKeysLit] = useState<Keys>(EMPTY_KEYS);
  const [nowTick, setNowTick] = useState(0);
  const [padPx, setPadPx] = useState(140);
  const [deckW, setDeckW] = useState(360);

  const keysRef = useRef<Keys>({ ...EMPTY_KEYS });
  const stickRef = useRef({ x: 0, y: 0 });
  const padRef = useRef<HTMLDivElement>(null);
  const rootRef = useRef<HTMLDivElement>(null);
  const dragging = useRef(false);
  const enabledRef = useRef(enabled);
  enabledRef.current = enabled;

  const odomCmd = useMemo(() => {
    const env =
      envelopes['/odom'] ??
      envelopes['odom'] ??
      envelopes['/Odometry'] ??
      envelopes['/localization/odometry'];
    const payload = envelopeJsonPayload(env);
    if (!payload) return null;
    const vx =
      readNumericPath(payload, 'twist.twist.linear.x') ??
      readNumericPath(payload, 'twist.linear.x') ??
      readNumericPath(payload, 'linear.x') ??
      readNumericPath(payload, 'vx');
    const wz =
      readNumericPath(payload, 'twist.twist.angular.z') ??
      readNumericPath(payload, 'twist.angular.z') ??
      readNumericPath(payload, 'angular.z') ??
      readNumericPath(payload, 'wz');
    if (vx == null && wz == null) return null;
    return { vx: vx ?? 0, wz: wz ?? 0 };
  }, [envelopes]);

  const clearMotion = useCallback(() => {
    stickRef.current = { x: 0, y: 0 };
    setStick({ x: 0, y: 0 });
    keysRef.current = { ...EMPTY_KEYS };
    setKeysLit({ ...EMPTY_KEYS });
    setHeld({ vx: 0, wz: 0 });
  }, []);

  const disarmAndStop = useCallback(() => {
    clearMotion();
    emergencyStop();
  }, [clearMotion]);

  const computeCmd = useCallback(() => {
    if (!enabledRef.current) return { vx: 0, wz: 0 };
    const { maxVx: mx, maxWz: mz } = useTeleopStore.getState().effectiveMax();
    const dz = useTeleopStore.getState().deadzone;
    const k = keysRef.current;
    let kx = 0;
    let ky = 0;
    if (k.w || k.up) ky -= 1;
    if (k.s || k.down) ky += 1;
    if (k.a || k.left) kx -= 1;
    if (k.d || k.right) kx += 1;
    const sx = applyDeadzone(stickRef.current.x, dz);
    const sy = applyDeadzone(stickRef.current.y, dz);
    if (Math.abs(sx) > 0.001 || Math.abs(sy) > 0.001) {
      return { vx: -sy * mx, wz: -sx * mz };
    }
    const mag = Math.hypot(kx, ky);
    if (mag > 1e-6) {
      kx /= mag;
      ky /= mag;
    }
    return { vx: -ky * mx, wz: -kx * mz };
  }, []);

  // Deck + stick scale with mosaic cell — fill most of the panel, stay cohesive.
  useEffect(() => {
    const root = rootRef.current;
    if (!root || typeof ResizeObserver === 'undefined') return;
    const ro = new ResizeObserver(() => {
      const w = root.clientWidth;
      const h = root.clientHeight;
      // Nearly full width; soft cap only for ultra-wide cells.
      const nextDeck = Math.round(Math.min(Math.max(260, w - 20), Math.min(560, w * 0.94)));
      // Leave room for status (~28) + gear/actions (~110) + padding.
      const usableH = Math.max(140, h - 150);
      const stickByW = nextDeck * (nextDeck < 340 ? 0.4 : 0.36);
      const stickByH = usableH * 0.55;
      const nextStick = Math.round(
        Math.min(260, Math.max(112, Math.min(stickByW, stickByH))),
      );
      setDeckW(nextDeck);
      setPadPx(nextStick);
      const scale = Math.min(1.35, Math.max(0.9, nextDeck / 360));
      root.style.setProperty('--teleop-scale', String(scale));
      root.style.setProperty('--teleop-deck-w', `${nextDeck}px`);
      root.style.setProperty('--teleop-stick', `${nextStick}px`);
    });
    ro.observe(root);
    return () => ro.disconnect();
  }, []);

  // Disconnect → disarm.
  useEffect(() => {
    if (!connected && enabled) disarmAndStop();
  }, [connected, enabled, disarmAndStop]);

  // Tab blur / hidden → disarm.
  useEffect(() => {
    const onBlur = () => {
      if (enabledRef.current) disarmAndStop();
    };
    const onVis = () => {
      if (document.visibilityState === 'hidden' && enabledRef.current) {
        disarmAndStop();
      }
    };
    window.addEventListener('blur', onBlur);
    document.addEventListener('visibilitychange', onVis);
    return () => {
      window.removeEventListener('blur', onBlur);
      document.removeEventListener('visibilitychange', onVis);
    };
  }, [disarmAndStop]);

  // cmd_vel_ack latency / echo.
  useEffect(() => {
    return wsClient.onMessage((msg) => {
      if (msg.op === 'cmd_vel_ack') {
        noteAck(msg.vx, msg.wz);
      }
    });
  }, [noteAck]);

  useEffect(() => {
    const id = window.setInterval(() => setNowTick((n) => n + 1), 250);
    return () => window.clearInterval(id);
  }, []);

  useEffect(() => {
    const syncLit = () => setKeysLit({ ...keysRef.current });
    const onKey = (e: KeyboardEvent, down: boolean) => {
      const tag = (e.target as HTMLElement)?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (!enabledRef.current && down) return;
      const k = e.key.toLowerCase();
      let handled = true;
      if (k === 'w') keysRef.current.w = down;
      else if (k === 'a') keysRef.current.a = down;
      else if (k === 's') keysRef.current.s = down;
      else if (k === 'd') keysRef.current.d = down;
      else if (e.key === 'ArrowUp') keysRef.current.up = down;
      else if (e.key === 'ArrowDown') keysRef.current.down = down;
      else if (e.key === 'ArrowLeft') keysRef.current.left = down;
      else if (e.key === 'ArrowRight') keysRef.current.right = down;
      else handled = false;
      if (handled) {
        e.preventDefault();
        syncLit();
      }
    };
    const down = (e: KeyboardEvent) => onKey(e, true);
    const up = (e: KeyboardEvent) => onKey(e, false);
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
    };
  }, []);

  // Soft-subscribe odom for cmd vs actual gauges.
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  useEffect(() => {
    if (!connected) return;
    for (const ch of ['/odom', 'odom', '/localization/odometry']) {
      wsClient.subscribe(ch, 20);
      markSubscribed(ch, 20);
    }
  }, [connected, markSubscribed]);

  useEffect(() => {
    if (!connected || !enabled) {
      setHeld({ vx: 0, wz: 0 });
      return;
    }
    const id = window.setInterval(() => {
      const cmd = computeCmd();
      setHeld(cmd);
      wsClient.cmdVel(cmd.vx, cmd.wz);
      noteSend();
    }, 1000 / SEND_HZ);
    return () => {
      window.clearInterval(id);
      emergencyStop();
      clearMotion();
    };
  }, [connected, enabled, computeCmd, noteSend, clearMotion]);

  const setFromPointer = (clientX: number, clientY: number) => {
    if (!enabledRef.current) return;
    const el = padRef.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    const nx = Math.max(-1, Math.min(1, (clientX - cx) / (r.width / 2)));
    const ny = Math.max(-1, Math.min(1, (clientY - cy) / (r.height / 2)));
    stickRef.current = { x: nx, y: ny };
    setStick({ x: nx, y: ny });
  };

  const ackAgeMs = lastAck ? Date.now() - lastAck.at : null;
  const sendAgeMs = lastSendAt ? Date.now() - lastSendAt : null;
  void nowTick;

  const linkTone =
    !connected
      ? 'bad'
      : ackAgeMs != null && ackAgeMs < 400
        ? 'ok'
        : enabled
          ? 'warn'
          : 'idle';

  const linkLabel = !connected
    ? connectionState === 'reconnecting'
      ? 'Reconnecting'
      : 'Offline'
    : ackAgeMs != null && ackAgeMs < 800
      ? `ACK ${ackAgeMs}ms`
      : enabled
        ? 'No ACK'
        : 'Idle';

  const knobTravel = padPx * 0.34;

  const onArm = () => {
    if (!connected) return;
    clearMotion();
    setEnabled(true);
  };

  const onDisarm = () => {
    disarmAndStop();
  };

  return (
    <div
      ref={rootRef}
      className={`panel teleop-panel${enabled ? ' is-armed' : ''}${!connected ? ' is-offline' : ''}`}
    >
      <div className="teleop-status">
        <span className={`teleop-pill tone-${linkTone}`}>
          {enabled ? 'Armed' : 'Disarmed'} · {linkLabel}
        </span>
        <span className="teleop-pill teleop-pill-muted mono">
          {CMD_CHANNEL} · {SEND_HZ}Hz
          {sendAgeMs != null && connected ? ` · tx ${sendAgeMs}ms` : ''}
        </span>
      </div>

      <div className="teleop-body">
        <div className="teleop-deck" style={{ width: deckW }}>
          <div className="teleop-drive">
            <div
              ref={padRef}
              className={`joystick${enabled && connected ? '' : ' is-disabled'}`}
              style={{ width: padPx, height: padPx }}
              onPointerDown={(e) => {
                if (!enabled || !connected) return;
                dragging.current = true;
                (e.currentTarget as HTMLElement).setPointerCapture?.(e.pointerId);
                setFromPointer(e.clientX, e.clientY);
              }}
              onPointerMove={(e) => {
                if (!dragging.current) return;
                setFromPointer(e.clientX, e.clientY);
              }}
              onPointerUp={() => {
                dragging.current = false;
                stickRef.current = { x: 0, y: 0 };
                setStick({ x: 0, y: 0 });
              }}
              onPointerCancel={() => {
                dragging.current = false;
                stickRef.current = { x: 0, y: 0 };
                setStick({ x: 0, y: 0 });
              }}
            >
              <div className="joystick-crosshair" />
              <div
                className="joystick-knob"
                style={{
                  width: Math.round(padPx * 0.28),
                  height: Math.round(padPx * 0.28),
                  transform: `translate(calc(-50% + ${stick.x * knobTravel}px), calc(-50% + ${stick.y * knobTravel}px))`,
                }}
              />
            </div>

            <div className="teleop-side">
              <div className="teleop-gauges">
                <Gauge
                  label="vx"
                  unit="m/s"
                  value={held.vx}
                  max={Math.max(0.05, maxVx)}
                  actual={odomCmd?.vx ?? null}
                  forwardPositive
                />
                <Gauge
                  label="wz"
                  unit="rad/s"
                  value={held.wz}
                  max={Math.max(0.05, maxWz)}
                  actual={odomCmd?.wz ?? null}
                  forwardPositive={false}
                />
              </div>
              <div className="teleop-wasd" aria-hidden>
                <span className={`teleop-key${keysLit.w || keysLit.up ? ' is-on' : ''}`}>W</span>
                <div className="teleop-wasd-row">
                  <span className={`teleop-key${keysLit.a || keysLit.left ? ' is-on' : ''}`}>A</span>
                  <span className={`teleop-key${keysLit.s || keysLit.down ? ' is-on' : ''}`}>S</span>
                  <span className={`teleop-key${keysLit.d || keysLit.right ? ' is-on' : ''}`}>D</span>
                </div>
              </div>
            </div>
          </div>

          <div className="teleop-gear">
            {(['slow', 'normal', 'fast'] as TeleopGear[]).map((g) => (
              <button
                key={g}
                type="button"
                className={`teleop-gear-btn${gear === g ? ' is-active' : ''}`}
                onClick={() => setGear(g)}
                title={`${TELEOP_GEAR_LABEL[g]} limit`}
              >
                {TELEOP_GEAR_LABEL[g]}
              </button>
            ))}
            <button
              type="button"
              className={`teleop-gear-btn teleop-settings-toggle${settingsOpen ? ' is-active' : ''}`}
              onClick={() => setSettingsOpen(!settingsOpen)}
              title="Limits & deadzone"
            >
              ⚙
            </button>
          </div>

          {settingsOpen ? (
            <div className="teleop-settings">
              <label>
                <span>max vx</span>
                <input
                  type="number"
                  step="0.05"
                  min="0.05"
                  max="3"
                  value={maxVxCfg}
                  onChange={(e) => setMaxVx(Number(e.target.value))}
                />
              </label>
              <label>
                <span>max wz</span>
                <input
                  type="number"
                  step="0.05"
                  min="0.05"
                  max="4"
                  value={maxWzCfg}
                  onChange={(e) => setMaxWz(Number(e.target.value))}
                />
              </label>
              <label>
                <span>deadzone</span>
                <input
                  type="number"
                  step="0.01"
                  min="0"
                  max="0.35"
                  value={deadzone}
                  onChange={(e) => setDeadzone(Number(e.target.value))}
                />
              </label>
              <p className="teleop-settings-note muted">
                Caps at Fast · gear scales down · Space = top-bar E-STOP
              </p>
            </div>
          ) : null}

          <div className="teleop-actions">
            {enabled ? (
              <button
                type="button"
                className="teleop-arm-btn is-on"
                onClick={onDisarm}
                disabled={!connected}
              >
                Disarm
              </button>
            ) : (
              <button
                type="button"
                className="teleop-arm-btn"
                onClick={onArm}
                disabled={!connected}
                title="Arm before driving"
              >
                Enable
              </button>
            )}
            <button
              type="button"
              className="teleop-stop-btn"
              onClick={disarmAndStop}
              disabled={!connected}
              title="Zero cmd_vel and disarm (panel STOP)"
            >
              STOP
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

function Gauge(props: {
  label: string;
  unit: string;
  value: number;
  max: number;
  actual: number | null;
  forwardPositive: boolean;
}) {
  const { label, unit, value, max, actual, forwardPositive } = props;
  const pct = Math.max(-1, Math.min(1, value / max));
  const width = `${Math.abs(pct) * 50}%`;
  const side = pct >= 0 ? 'pos' : 'neg';
  return (
    <div className="teleop-gauge">
      <div className="teleop-gauge-head">
        <span className="teleop-gauge-label">{label}</span>
        <span className={`teleop-gauge-val mono${Math.abs(value) > 0.01 ? ' is-live' : ''}`}>
          {value >= 0 ? '+' : ''}
          {value.toFixed(2)}
          <span className="muted"> {unit}</span>
        </span>
      </div>
      <div className="teleop-gauge-track">
        <div className="teleop-gauge-mid" />
        <div
          className={`teleop-gauge-fill ${side}${forwardPositive ? ' is-linear' : ' is-angular'}`}
          style={
            pct >= 0
              ? { left: '50%', width }
              : { right: '50%', width, left: 'auto' }
          }
        />
      </div>
      {actual != null ? (
        <div className="teleop-gauge-actual muted mono">
          odom {actual >= 0 ? '+' : ''}
          {actual.toFixed(2)}
        </div>
      ) : null}
    </div>
  );
}
