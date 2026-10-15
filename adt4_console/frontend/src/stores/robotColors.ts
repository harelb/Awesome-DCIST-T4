// Loads per-robot colours (GET /api/network/colors) and tracks live robot names
// for the palette fallback; see lib/palette.ts robotColor.
import {getJSON} from "../lib/api";
import {useRobotColors} from "../lib/palette";
import {ws} from "../ws";
import {useRobots} from "./robots";

export async function loadRobotColors(): Promise<void> {
  try {
    const r = await getJSON<{colors: Record<string, string>}>("api/network/colors");
    if (r?.colors) useRobotColors.setState({colors: r.colors});
  } catch {
    // keep the palette fallback; colours are cosmetic
  }
}

function liveNames(): string[] {
  const s = useRobots.getState();
  return [...new Set([...Object.keys(s.state), ...Object.keys(s.link), ...Object.keys(s.plan), ...Object.keys(s.paths)])].sort();
}

export function bindRobotColors() {
  void loadRobotColors();
  ws.onReconnect(() => void loadRobotColors());
  // topology edits (rename/add machine) change the colour map; only refetch when
  // the machine set changes, not on every reachability update
  let machineKey = "";
  ws.on("fleet/machines", d => {
    const key = ((d as {machines?: {name: string}[]})?.machines ?? []).map(m => m.name).sort().join(",");
    if (key !== machineKey) {
      machineKey = key;
      void loadRobotColors();
    }
  });
  useRobots.subscribe(() => {
    const names = liveNames();
    if (names.join(",") !== useRobotColors.getState().names.join(",")) useRobotColors.setState({names});
  });
}
