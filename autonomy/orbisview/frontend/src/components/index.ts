import { registerPanel } from './registry';
import { Map2DPanel } from './Map2D/Map2DPanel';
import { View3DPanel } from './View3D/View3DPanel';
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
  registerPanel({ id: 'map2d', title: 'Map 2D', component: Map2DPanel });
  registerPanel({ id: 'dashboard', title: 'Dashboard', component: DashboardPanel });
  registerPanel({ id: 'mode_settings', title: 'Mode Settings', component: ModeSettingsPanel });
  registerPanel({ id: 'module_delay', title: 'Module Delay', component: ModuleDelayPanel });
  registerPanel({ id: 'resources', title: 'Resources', component: ResourceManagerPanel });
  registerPanel({ id: 'robot_status', title: 'Robot Status', component: RobotStatusPanel });
  registerPanel({ id: 'tf_tree', title: 'TF Tree', component: TfTreePanel });
  registerPanel({ id: 'waypoints', title: 'Waypoints', component: WaypointsPanel });
  registerPanel({ id: 'routing', title: 'Routing', component: RoutingPanel });
  registerPanel({ id: 'teleop', title: 'Teleop', component: TeleopPanel });
  registerPanel({ id: 'view3d', title: 'View 3D', component: View3DPanel });
  registerPanel({ id: 'image', title: 'Image', component: ImagePanel });
  registerPanel({ id: 'log', title: 'Channels', component: ChannelLogPanel });
  registerPanel({ id: 'inspector', title: 'Inspector', component: InspectorPanel });
  registerPanel({ id: 'diagnostics', title: 'Diagnostics', component: DiagnosticsPanel });
  registerPanel({ id: 'stats', title: 'Stats', component: StatsPanel });
  registerPanel({ id: 'recorder', title: 'Recorder', component: RecorderPanel });
  registerPanel({ id: 'plugins', title: 'Plugins', component: PluginsPanel });
  registerPanel({ id: 'pnc', title: 'PNC Monitor', component: PncMonitorPanel });
  registerPanel({ id: 'charts', title: 'Charts', component: ChartsPanel });
  registerPanel({ id: 'components', title: 'Components', component: ComponentsPanel });
  registerPanel({ id: 'hmi', title: 'HMI', component: HmiPanel });
  registerPanel({ id: 'exploration', title: 'Exploration', component: ExplorationPanel });
  registerPanel({ id: 'navigation', title: 'Navigation', component: NavigationPanel });
  registerPanel({ id: 'mapping', title: 'Mapping', component: MappingPanel });
}
