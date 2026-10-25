// The mission planner's own map layers: on /mission the places layers (mesh
// places, traversability) are off by default so the satellite, the objects and
// the mission read clearly; the operator's layer changes there are remembered
// per browser (viewStyle.missionHidden) and the robot viewer's layer set comes
// back on leaving the page.
import {useViewer} from "./viewer";
import {useViewStyle} from "./viewStyle";

/** /mission defaults: sub-keyframes (as everywhere) and every places layer. */
export const MISSION_HIDDEN_DEFAULT: Record<string, boolean> = {"2/115": true, "kind:places": true};

let saved: Record<string, boolean> | null = null;

/** The mission planner's layer set: its defaults under the operator's overrides. */
export function missionHidden(overrides: Record<string, boolean>): Record<string, boolean> {
  return {...MISSION_HIDDEN_DEFAULT, ...overrides};
}

export function enterMissionView() {
  if (saved === null) saved = useViewer.getState().hiddenLayers;
  useViewer.getState().set({hiddenLayers: missionHidden(useViewStyle.getState().missionHidden)});
}

export function leaveMissionView() {
  if (saved === null) return;
  useViewer.getState().set({hiddenLayers: saved});
  saved = null;
}

/** Layers panel: show / hide one layer ("layer/part"); on /mission the choice is kept. */
export function setLayerHidden(key: string, hidden: boolean) {
  const v = useViewer.getState();
  v.set({hiddenLayers: {...v.hiddenLayers, [key]: hidden}});
  if (saved !== null) {
    const st = useViewStyle.getState();
    st.patch({missionHidden: {...st.missionHidden, [key]: hidden}});
  }
}
