// The omniplanner engines this console can plan with (API.md §15.5):
// Fast Downward (shortest routes) and GRSTAPS-X (task allocation + a timed
// schedule; needs the itags solver and a Gurobi license on the console's machine).
import {create} from "zustand";
import {getJSON} from "../lib/api";

export type EngineId = "fast_downward" | "grstaps";
export interface EngineInfo {
  id: EngineId;
  label: string;
  available: boolean;
  reason: string | null;
}

interface EnginesStore {
  engines: EngineInfo[];
  /** the operator's choice for omniplanner plans (mission tab, goal previews) */
  engine: EngineId;
  loaded: boolean;
  setEngine: (e: EngineId) => void;
}

export const ENGINE_LABEL: Record<EngineId, string> = {fast_downward: "Fast Downward", grstaps: "GRSTAPS-X"};

const ENGINE_KEY = "adt4.omniplanner.engine";

function loadEngine(): EngineId {
  try {
    return localStorage.getItem(ENGINE_KEY) === "grstaps" ? "grstaps" : "fast_downward";
  } catch {
    return "fast_downward";
  }
}

export const useEngines = create<EnginesStore>(set => ({
  engines: [],
  engine: loadEngine(),
  loaded: false,
  setEngine: engine => {
    try {
      localStorage.setItem(ENGINE_KEY, engine);
    } catch {
      /* the choice just isn't remembered */
    }
    set({engine});
  },
}));

/** (Re)load the engines and whether each can plan here (a license copied in later turns GRSTAPS-X on). */
export async function refreshEngines(): Promise<EngineInfo[]> {
  try {
    const r = await getJSON<{engines: EngineInfo[]; default: EngineId}>("api/missions/plan/engines");
    const cur = useEngines.getState().engine;
    const ok = r.engines.find(e => e.id === cur)?.available;
    useEngines.setState({engines: r.engines, loaded: true, ...(ok ? {} : {engine: r.default})});
    return r.engines;
  } catch {
    useEngines.setState({loaded: true});
    return [];
  }
}
