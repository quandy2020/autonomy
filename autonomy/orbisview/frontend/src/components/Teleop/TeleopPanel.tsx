import { useCallback, useEffect, useRef, useState } from 'react';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';
import { emergencyStop } from './emergencyStop';

const MAX_VX = 0.8;
const MAX_WZ = 1.2;
const SEND_HZ = 15;

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

function keysToCmd(k: Keys): { vx: number; wz: number } {
  let vx = 0;
  let wz = 0;
  if (k.w || k.up) vx += MAX_VX;
  if (k.s || k.down) vx -= MAX_VX;
  if (k.a || k.left) wz += MAX_WZ;
  if (k.d || k.right) wz -= MAX_WZ;
  return { vx, wz };
}

/** Mosaic teleop panel: stick + WASD / arrows while mounted. */
export function TeleopPanel() {
  const connected = useDataStore((s) => s.connected);
  const [stick, setStick] = useState({ x: 0, y: 0 });
  const [held, setHeld] = useState({ vx: 0, wz: 0 });
  const keysRef = useRef<Keys>({
    w: false,
    a: false,
    s: false,
    d: false,
    up: false,
    down: false,
    left: false,
    right: false,
  });
  const stickRef = useRef({ x: 0, y: 0 });
  const padRef = useRef<HTMLDivElement>(null);
  const dragging = useRef(false);

  const computeCmd = useCallback(() => {
    const fromKeys = keysToCmd(keysRef.current);
    const sx = stickRef.current.x;
    const sy = stickRef.current.y;
    const fromStick = {
      vx: -sy * MAX_VX,
      wz: -sx * MAX_WZ,
    };
    if (Math.abs(sx) > 0.02 || Math.abs(sy) > 0.02) return fromStick;
    return fromKeys;
  }, []);

  const stop = useCallback(() => {
    stickRef.current = { x: 0, y: 0 };
    setStick({ x: 0, y: 0 });
    keysRef.current = {
      w: false,
      a: false,
      s: false,
      d: false,
      up: false,
      down: false,
      left: false,
      right: false,
    };
    setHeld({ vx: 0, wz: 0 });
    emergencyStop();
  }, []);

  useEffect(() => {
    const onKey = (e: KeyboardEvent, down: boolean) => {
      const tag = (e.target as HTMLElement)?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
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
      if (handled) e.preventDefault();
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

  useEffect(() => {
    if (!connected) return;
    const id = window.setInterval(() => {
      const cmd = computeCmd();
      setHeld(cmd);
      wsClient.cmdVel(cmd.vx, cmd.wz);
    }, 1000 / SEND_HZ);
    return () => {
      window.clearInterval(id);
      emergencyStop();
    };
  }, [connected, computeCmd]);

  const setFromPointer = (clientX: number, clientY: number) => {
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

  return (
    <div className="panel teleop-panel">
      <div className="teleop-panel-body">
        <div
          ref={padRef}
          className="joystick"
          onPointerDown={(e) => {
            dragging.current = true;
            (e.target as HTMLElement).setPointerCapture?.(e.pointerId);
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
          <div
            className="joystick-knob"
            style={{
              transform: `translate(calc(-50% + ${stick.x * 36}px), calc(-50% + ${stick.y * 36}px))`,
            }}
          />
        </div>
        <div className="teleop-meta">
          <div className="teleop-keys">WASD / arrows</div>
          <div className="muted teleop-vel">
            vx={held.vx.toFixed(2)} wz={held.wz.toFixed(2)}
            {!connected ? ' · offline' : ''}
          </div>
          <p className="muted teleop-hint">Space · top-bar E-STOP</p>
          <button type="button" className="estop" onClick={stop} disabled={!connected}>
            E-STOP
          </button>
        </div>
      </div>
    </div>
  );
}
