import { ModeSettingsPanel } from '@/components/Mode/ModePanels';
import { IconLabel } from '@/components/icons';
import { defaultWsUrl } from '@/config/parameters';

/** Settings pane: mode/modules. */
export function SidebarSetting() {
  return (
    <div className="sidebar-section sidebar-setting">
      <h3 className="sidebar-section-title">
        <IconLabel name="setting" label="Setting" size={13} />
      </h3>
      <p className="hint sidebar-hint">
        WS 默认 <code>{defaultWsUrl()}</code>
      </p>
      <ModeSettingsPanel />
    </div>
  );
}
