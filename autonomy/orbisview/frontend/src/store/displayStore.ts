import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import {
  defaultPropsFor,
  getDisplayTypeDef,
  resolveDisplayType,
  type PropValue,
} from '@/components/Channels/displayTypes';

export interface ChannelDisplay {
  id: string;
  typeId: string;
  enabled: boolean;
  channel: string;
  maxHz: number;
  props: Record<string, PropValue>;
}

interface DisplayState {
  displays: ChannelDisplay[];
  selectedId: string | null;
  addDisplay: (packageName: string, message: string, channel?: string) => string;
  removeDisplay: (id: string) => void;
  selectDisplay: (id: string | null) => void;
  setEnabled: (id: string, enabled: boolean) => void;
  setChannel: (id: string, channel: string) => void;
  setMaxHz: (id: string, maxHz: number) => void;
  setProp: (id: string, key: string, value: PropValue) => void;
  clear: () => void;
}

export const useDisplayStore = create<DisplayState>()(
  persist(
    (set, get) => ({
      displays: [],
      selectedId: null,
      addDisplay: (packageName, message, channel = '') => {
        const def = resolveDisplayType(packageName, message);
        const id = `disp-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`;
        const props = defaultPropsFor(def, def.label);
        if (channel) props.topic = channel;
        const item: ChannelDisplay = {
          id,
          typeId: def.typeId,
          enabled: true,
          channel,
          maxHz: 20,
          props,
        };
        set({ displays: [...get().displays, item], selectedId: id });
        return id;
      },
      removeDisplay: (id) => {
        const displays = get().displays.filter((d) => d.id !== id);
        const selectedId = get().selectedId === id ? displays[0]?.id ?? null : get().selectedId;
        set({ displays, selectedId });
      },
      selectDisplay: (id) => set({ selectedId: id }),
      setEnabled: (id, enabled) =>
        set({
          displays: get().displays.map((d) => (d.id === id ? { ...d, enabled } : d)),
        }),
      setChannel: (id, channel) =>
        set({
          displays: get().displays.map((d) =>
            d.id === id ? { ...d, channel, props: { ...d.props, topic: channel } } : d,
          ),
        }),
      setMaxHz: (id, maxHz) =>
        set({
          displays: get().displays.map((d) =>
            d.id === id
              ? { ...d, maxHz, props: { ...d.props, maxHz } }
              : d,
          ),
        }),
      setProp: (id, key, value) =>
        set({
          displays: get().displays.map((d) => {
            if (d.id !== id) return d;
            const props = { ...d.props, [key]: value };
            const patch: Partial<ChannelDisplay> = { props };
            if (key === 'topic' && typeof value === 'string') patch.channel = value;
            if (key === 'enabled' && typeof value === 'boolean') patch.enabled = value;
            if (key === 'maxHz' && typeof value === 'number') patch.maxHz = value;
            return { ...d, ...patch };
          }),
        }),
      clear: () => set({ displays: [], selectedId: null }),
    }),
    {
      name: 'orbisview-displays-v1',
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<DisplayState>;
        const displays = (p.displays ?? []).filter((d) => !!getDisplayTypeDef(d.typeId) || !!d.typeId);
        return { ...current, ...p, displays };
      },
    },
  ),
);
