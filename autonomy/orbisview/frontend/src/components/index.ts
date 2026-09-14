import { registerPanel } from './registry';
import { MapPanel } from './Map/MapPanel';
import { ImagePanel } from './Image/ImagePanel';
import { ChannelLogPanel } from './ChannelLog/ChannelLogPanel';
import { InspectorPanel } from './Inspector/InspectorPanel';
import { DiagnosticsPanel } from './Diagnostics/DiagnosticsPanel';
import { StatsPanel } from './Stats/StatsPanel';
import { RecorderPanel } from './Recorder/RecorderPanel';
import { PluginsPanel } from './Plugins/PluginsPanel';
import { PncMonitorPanel } from './Pnc/PncMonitorPanel';
import {
  ExplorationPanel,
  NavigationPanel,
  MappingPanel,
} from './Tasks/TaskPanels';
import { RobotStatusPanel } from './RobotStatus/RobotStatusPanel';
import { TfTreePanel } from './RobotStatus/TfTreePanel';
import { WaypointsPanel } from './Waypoints/WaypointsPanel';
import { TeleopPanel } from './Teleop/TeleopPanel';
import { OpsPanel } from './Ops/OpsPanel';
import { DashboardPanel } from './Dashboard/DashboardPanel';
import { ComponentsPanel, HmiPanel } from './Hmi/HmiPanels';
import { ChartsPanel } from './Charts/ChartsPanel';
import { RoutingPanel } from './Routing/RoutingPanel';
import {
  ModeSettingsPanel,
  ModuleDelayPanel,
  ResourceManagerPanel,
} from './Mode/ModePanels';

export function registerBuiltinPanels(): void {
  // 可视化
  registerPanel({ id: 'map', title: 'Map', category: 'viz', component: MapPanel });

  // 传感器
  registerPanel({
    id: 'image',
    title: 'Image',
    category: 'sensor',
    allowMultiple: true,
    component: ImagePanel,
  });
  registerPanel({ id: 'log', title: 'Channels', category: 'sensor', component: ChannelLogPanel });

  // 机器人
  registerPanel({
    id: 'dashboard',
    title: 'Dashboard',
    category: 'robot',
    component: DashboardPanel,
  });
  registerPanel({
    id: 'robot_status',
    title: 'Robot Status',
    category: 'robot',
    component: RobotStatusPanel,
  });
  registerPanel({ id: 'tf_tree', title: 'TF Tree', category: 'robot', component: TfTreePanel });
  registerPanel({ id: 'teleop', title: 'Teleop', category: 'robot', component: TeleopPanel });

  // 规划任务
  registerPanel({
    id: 'waypoints',
    title: 'Waypoints',
    category: 'planning',
    component: WaypointsPanel,
  });
  registerPanel({ id: 'routing', title: 'Routing', category: 'planning', component: RoutingPanel });
  registerPanel({
    id: 'navigation',
    title: 'Navigation',
    category: 'planning',
    component: NavigationPanel,
  });
  registerPanel({
    id: 'exploration',
    title: 'Exploration',
    category: 'planning',
    component: ExplorationPanel,
  });
  registerPanel({ id: 'mapping', title: 'Mapping', category: 'planning', component: MappingPanel });
  registerPanel({ id: 'pnc', title: 'PNC Monitor', category: 'planning', component: PncMonitorPanel });

  // 监控
  registerPanel({
    id: 'diagnostics',
    title: 'Diagnostics',
    category: 'monitor',
    component: DiagnosticsPanel,
  });
  registerPanel({ id: 'stats', title: 'Stats', category: 'monitor', component: StatsPanel });
  registerPanel({ id: 'charts', title: 'Charts', category: 'monitor', component: ChartsPanel });
  registerPanel({
    id: 'inspector',
    title: 'Inspector',
    category: 'monitor',
    component: InspectorPanel,
  });
  registerPanel({
    id: 'module_delay',
    title: 'Module Delay',
    category: 'monitor',
    component: ModuleDelayPanel,
  });

  // 系统
  registerPanel({
    id: 'mode_settings',
    title: 'Mode Settings',
    category: 'system',
    component: ModeSettingsPanel,
  });
  registerPanel({
    id: 'resources',
    title: 'Resources',
    category: 'system',
    component: ResourceManagerPanel,
  });
  registerPanel({ id: 'recorder', title: 'Recorder', category: 'system', component: RecorderPanel });
  registerPanel({ id: 'ops', title: 'Ops', category: 'system', component: OpsPanel });
  registerPanel({ id: 'plugins', title: 'Plugins', category: 'system', component: PluginsPanel });
  registerPanel({
    id: 'components',
    title: 'Components',
    category: 'system',
    component: ComponentsPanel,
  });
  registerPanel({ id: 'hmi', title: 'HMI', category: 'system', component: HmiPanel });
}
