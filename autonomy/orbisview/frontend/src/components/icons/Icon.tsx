import type { ReactNode, SVGProps } from 'react';

export type IconName =
  | 'orbis'
  | 'plug'
  | 'unplug'
  | 'catalog'
  | 'preset'
  | 'mode'
  | 'panels'
  | 'resources'
  | 'layers'
  | 'teleop'
  | 'pnc'
  | 'ops'
  | 'map2d'
  | 'view3d'
  | 'dashboard'
  | 'robot'
  | 'tf'
  | 'waypoint'
  | 'route'
  | 'image'
  | 'channels'
  | 'inspector'
  | 'diagnostics'
  | 'stats'
  | 'recorder'
  | 'plugins'
  | 'charts'
  | 'components'
  | 'hmi'
  | 'explore'
  | 'nav'
  | 'mapping'
  | 'delay'
  | 'dump'
  | 'clear'
  | 'play'
  | 'pause'
  | 'stop'
  | 'statusOnline'
  | 'statusWarn'
  | 'statusOffline'
  | 'plus'
  | 'minus'
  | 'zoomIn'
  | 'zoomOut'
  | 'fit'
  | 'measure'
  | 'pick'
  | 'send'
  | 'follow'
  | 'pan'
  | 'goal'
  | 'laser'
  | 'costmap'
  | 'path'
  | 'footprint'
  | 'obstacle'
  | 'prediction'
  | 'grid'
  | 'pointcloud'
  | 'depth'
  | 'pose'
  | 'velocity'
  | 'chassis'
  | 'json'
  | 'msg'
  | 'audio'
  | 'mesh'
  | 'task'
  | 'setting'
  | 'sidebar'
  | 'layout';

const PATHS: Record<IconName, string> = {
  orbis:
    'M12 3a9 9 0 1 0 9 9 M12 7a5 5 0 1 0 5 5 M16.5 5.5c2.2 1.4 3.5 3.6 3.5 6.5',
  plug: 'M9 7v4 M15 7v4 M8 11h8v3a4 4 0 0 1-8 0v-3z M12 18v3',
  unplug: 'M9 7v3 M15 7v3 M8 10h8v2a4 4 0 0 1-3 3.87 M12 16v2 M5 5l14 14',
  catalog: 'M4 6h16 M4 12h16 M4 18h10',
  preset: 'M4 20V10l8-6 8 6v10 M9 20v-6h6v6',
  mode: 'M12 3l8 4.5v9L12 21l-8-4.5v-9L12 3z M12 12l8-4.5 M12 12v9 M12 12L4 7.5',
  panels: 'M3 3h8v8H3z M13 3h8v5h-8z M13 10h8v11h-8z M3 13h8v8H3z',
  resources: 'M4 7h16v12H4z M8 7V5h8v2 M9 12h6',
  layers: 'M12 3l9 5-9 5-9-5 9-5z M3 13l9 5 9-5 M3 17l9 5 9-5',
  teleop:
    'M12 8v8 M8 12h8 M7 7l2 2 M17 7l-2 2 M7 17l2-2 M17 17l-2-2 M12 4a8 8 0 1 1 0 16 8 8 0 0 1 0-16z',
  pnc: 'M4 18V6h4l3 6 3-6h4v12h-3V11l-3 5h-2l-3-5v7H4z',
  ops: 'M14.7 6.3a1 1 0 0 0 0 1.4l1.6 1.6a1 1 0 0 0 1.4 0l3.8-3.8a8 8 0 1 1-4.9-2.1 M16 8l4-4',
  map2d: 'M3 6l6-3 6 3 6-3v15l-6 3-6-3-6 3V6z M9 3v15 M15 6v15',
  view3d: 'M12 3l9 5v8l-9 5-9-5V8l9-5z M12 12l9-5 M12 12v10 M12 12L3 7',
  dashboard: 'M4 13a8 8 0 0 1 16 0 M12 13V8 M8 16h.01 M16 16h.01',
  robot:
    'M9 9V7a3 3 0 0 1 6 0v2 M7 9h10v8a3 3 0 0 1-3 3H10a3 3 0 0 1-3-3V9z M10 13h.01 M14 13h.01',
  tf: 'M5 7h6v6H5z M13 11h6v6h-6z M11 10l4 3',
  waypoint: 'M12 21s-7-5.5-7-11a7 7 0 1 1 14 0c0 5.5-7 11-7 11z M12 10a2 2 0 1 0 0-4 2 2 0 0 0 0 4z',
  route: 'M4 17h4a3 3 0 0 0 3-3V7a3 3 0 0 1 3-3h6 M17 4l3 3-3 3',
  image: 'M4 5h16v14H4z M8 11a2 2 0 1 0 0-4 2 2 0 0 0 0 4z M4 15l5-4 3 3 3-2 5 4',
  channels: 'M4 6h16 M4 12h10 M4 18h14 M16 10v4 M20 10v4',
  inspector: 'M11 5a6 6 0 1 0 3.5 10.7L18 19l2-2-3.3-3.5A6 6 0 0 0 11 5z',
  diagnostics: 'M12 3v3 M12 18v3 M5 12H2 M22 12h-3 M6.3 6.3l-2-2 M19.7 19.7l-2-2 M6.3 17.7l-2 2 M19.7 4.3l-2 2 M9 12a3 3 0 1 0 6 0 3 3 0 0 0-6 0z',
  stats: 'M4 19V9 M10 19V5 M16 19v-7 M22 19V8',
  recorder: 'M12 12a4 4 0 1 0 0-8 4 4 0 0 0 0 8z M12 14c-4 0-7 1.5-7 3.5V20h14v-2.5c0-2-3-3.5-7-3.5z',
  plugins: 'M12 2v4 M12 18v4 M4.9 4.9l2.8 2.8 M16.3 16.3l2.8 2.8 M2 12h4 M18 12h4 M4.9 19.1l2.8-2.8 M16.3 7.7l2.8-2.8 M9 9h6v6H9z',
  charts: 'M4 19h16 M7 16V9 M12 16V5 M17 16v-4',
  components: 'M7 7h3v3H7z M14 7h3v3h-3z M7 14h3v3H7z M14 14h3v3h-3z M10 8.5h4 M8.5 10v4 M15.5 10v4 M10 15.5h4',
  hmi: 'M4 6h16v10H4z M8 20h8 M12 16v4',
  explore: 'M12 3a9 9 0 1 0 9 9 M12 8v4l3 2 M16 5l2-2 2 2-2 2',
  nav: 'M12 2l4 8H8l4-8z M5 14h14v6H5z',
  mapping: 'M9 3H5v4 M15 3h4v4 M9 21H5v-4 M15 21h4v-4 M8 12h8 M12 8v8',
  delay: 'M12 7v5l3 2 M12 3a9 9 0 1 0 9 9',
  dump: 'M12 3v10 M8 9l4 4 4-4 M5 17h14v4H5z',
  clear: 'M4 7h16 M9 7V5h6v2 M7 7l1 12h8l1-12',
  play: 'M8 5v14l11-7z',
  pause: 'M7 5h3v14H7z M14 5h3v14h-3z',
  stop: 'M6 6h12v12H6z',
  statusOnline: 'M9 12a3 3 0 1 0 6 0 3 3 0 1 0-6 0',
  statusWarn: 'M12 8v5 M12 16h.01 M10.3 4.3 2.6 18a2 2 0 0 0 1.7 3h15.4a2 2 0 0 0 1.7-3L13.7 4.3a2 2 0 0 0-3.4 0z',
  statusOffline: 'M5 5l14 14 M12 4a8 8 0 0 1 8 8 8 8 0 0 1-2.3 5.6 M4.3 7.5A8 8 0 0 0 12 20',
  plus: 'M12 5v14 M5 12h14',
  minus: 'M5 12h14',
  zoomIn: 'M11 5a6 6 0 1 0 3.5 10.7L18 19l2-2-3.3-3.5A6 6 0 0 0 11 5z M8 11h6 M11 8v6',
  zoomOut: 'M11 5a6 6 0 1 0 3.5 10.7L18 19l2-2-3.3-3.5A6 6 0 0 0 11 5z M8 11h6',
  fit: 'M4 9V4h5 M15 4h5v5 M20 15v5h-5 M9 20H4v-5',
  measure: 'M4 20l16-16 M7 20h.01 M10 17h.01 M13 14h.01 M16 11h.01 M19 8h.01 M4 7h3v3',
  pick: 'M12 3v7 M9 7l3 3 3-3 M5 14h14v6H5z',
  send: 'M22 2L11 13 M22 2l-7 20-4-9-9-4 20-7z',
  follow: 'M12 12a3 3 0 1 0 0-6 3 3 0 0 0 0 6z M4 20c1.5-3 4-5 8-5s6.5 2 8 5',
  pan: 'M8 11V7a2 2 0 1 1 4 0v4 M12 11V5a2 2 0 1 1 4 0v6 M16 11V8a2 2 0 1 1 4 0v7a5 5 0 0 1-5 5h-2a7 7 0 0 1-7-7v-2a2 2 0 1 1 4 0',
  goal: 'M12 3l3 7h7l-5.5 4.5L19 22l-7-4-7 4 1.5-7.5L1 10h7z',
  laser: 'M12 3v6 M12 15v6 M5 12H3 M21 12h-2 M7 7l-1.5-1.5 M18.5 18.5 17 17 M7 17l-1.5 1.5 M18.5 5.5 17 7',
  costmap: 'M4 4h16v16H4z M8 8h3v3H8z M13 13h3v3h-3z M8 13h2v2H8z',
  path: 'M4 18c4-2 4-10 8-12s4 4 8 2',
  footprint: 'M8 7h8v10H8z M12 7V4 M9 17v3 M15 17v3',
  obstacle: 'M7 8h10v10H7z M10 11h4',
  prediction: 'M4 18c3-6 6-8 8-8s5 2 8 8 M12 10V6',
  grid: 'M4 4h16v16H4z M4 10h16 M4 16h16 M10 4v16 M16 4v16',
  pointcloud:
    'M7 8a1 1 0 1 0 0.01 0 M12 6a1 1 0 1 0 0.01 0 M17 8a1 1 0 1 0 0.01 0 M6 13a1 1 0 1 0 0.01 0 M12 12a1 1 0 1 0 0.01 0 M18 13a1 1 0 1 0 0.01 0 M9 17a1 1 0 1 0 0.01 0 M15 17a1 1 0 1 0 0.01 0',
  depth: 'M4 8c4 6 12 6 16 0 M4 12c4 6 12 6 16 0 M4 16c4 6 12 6 16 0',
  pose: 'M12 12a3 3 0 1 0 0-6 3 3 0 0 0 0 6z M12 12l5 7 M9.5 14.5 7 20 M14.5 14.5 17 20',
  velocity: 'M4 16c3-8 6-11 8-11s5 3 8 11 M12 5v3',
  chassis: 'M5 14h14l-1.5-5H6.5L5 14z M7 14v3 M17 14v3 M8.5 11.5h7',
  json: 'M8 4h8v2H8z M7 8h10v12H7z M10 12h4 M10 15h6',
  msg: 'M5 5h14v10H9l-4 4V5z',
  audio: 'M4 10v4 M8 7v10 M12 4v16 M16 7v10 M20 10v4',
  mesh: 'M4 7l8-4 8 4v10l-8 4-8-4V7z M12 3v18 M4 7l8 4 8-4',
  task: 'M9 6h11 M9 12h11 M9 18h11 M4.5 6h.01 M4.5 12h.01 M4.5 18h.01 M5 5.5l1.2 1.2L8 4.8',
  setting:
    'M12 8.5a3.5 3.5 0 1 0 0 7 3.5 3.5 0 0 0 0-7z M12 3v2.2 M12 18.8V21 M4.9 6.5l1.6 1.6 M17.5 15.9l1.6 1.6 M3 12h2.2 M18.8 12H21 M4.9 17.5l1.6-1.6 M17.5 8.1l1.6-1.6',
  sidebar: 'M4 5h16v14H4z M9 5v14',
  layout: 'M4 5h16v14H4z M4 10h16 M10 10v9',
};

export interface IconProps extends Omit<SVGProps<SVGSVGElement>, 'name'> {
  name: IconName;
  size?: number | string;
  title?: string;
}

/** Stroke-based ops icons; inherit color via currentColor. */
export function Icon({ name, size = 16, title, className, ...rest }: IconProps) {
  const d = PATHS[name];
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.75}
      strokeLinecap="round"
      strokeLinejoin="round"
      className={className ? `ov-icon ${className}` : 'ov-icon'}
      aria-hidden={title ? undefined : true}
      role={title ? 'img' : undefined}
      {...rest}
    >
      {title ? <title>{title}</title> : null}
      <path d={d} />
    </svg>
  );
}

export function IconLabel({
  name,
  label,
  children,
  size = 14,
}: {
  name: IconName;
  label?: string;
  children?: ReactNode;
  size?: number;
}) {
  return (
    <span className="ov-icon-label">
      <Icon name={name} size={size} />
      <span>{children ?? label}</span>
    </span>
  );
}
