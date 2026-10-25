// Mission planner / map-edit keyboard shortcuts (listed in the "?" help,
// controlsHelp.ts MISSION_HELP): Ctrl/Cmd+Z undoes the newest start move or map
// edit (stores/tools.ts undoLatest), Delete removes the edit selection (like its
// Remove button: undoable from the toast / Ctrl+Z), F fits the mission, Space
// plays / pauses the simulation. The drawing keys (Enter / Backspace / Esc) are
// ToolBanner's. None of them fire while typing in a field; Space also leaves a
// focused button alone (it presses it).
import {useEffect} from "react";
import {fitMission, simCmd, useMission} from "../stores/mission";
import {deleteSelection, useMapEdit} from "../stores/mapEdit";
import {undoLatest, undoTarget} from "../stores/tools";

export const isTyping = (t: EventTarget | null) => {
  const el = t as HTMLElement | null;
  return !!el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName));
};

export type Shortcut = "undo" | "delete" | "fit" | "play";

/** The shortcut a key press asks for (null: not one of ours, or typing). */
export function shortcutOf(e: Pick<KeyboardEvent, "key" | "ctrlKey" | "metaKey" | "altKey" | "shiftKey" | "target">): Shortcut | null {
  if (isTyping(e.target)) return null;
  const mod = e.ctrlKey || e.metaKey;
  if (mod && !e.altKey && !e.shiftKey && e.key.toLowerCase() === "z") return "undo";
  if (mod || e.altKey) return null;
  if (e.key === "Delete") return "delete";
  if (e.key === "f" || e.key === "F") return "fit";
  if (e.key === " " && !(e.target instanceof HTMLButtonElement)) return "play";
  return null;
}

/** Space: start the simulation, or pause / resume / replay it (not a live run). */
export async function toggleSim(): Promise<boolean> {
  const {current, preview, run} = useMission.getState();
  if (!current || run?.kind === "live") return false;
  if (!run) {
    if (preview?.issues.some(i => i.level === "error")) return false;
    await simCmd("new");
    await simCmd("play");
    fitMission();
    return true;
  }
  if (run.playing) await simCmd("pause");
  else if (run.finished) {
    await simCmd("reset");
    await simCmd("play");
  } else await simCmd("play");
  return true;
}

/** Run a shortcut; false when it does not apply now (the key stays the browser's). */
export function runShortcut(s: Shortcut): boolean {
  switch (s) {
    case "undo":
      if (!undoTarget(useMission.getState().startUndo, useMapEdit.getState().editUndo)) return false;
      void undoLatest();
      return true;
    case "delete": {
      const e = useMapEdit.getState();
      if (!e.on || !e.selection.length || e.busy) return false;
      void deleteSelection();
      return true;
    }
    case "fit":
      return fitMission();
    case "play": {
      const {current, run} = useMission.getState();
      if (!current || run?.kind === "live") return false;
      void toggleSim();
      return true;
    }
  }
}

export function useMapShortcuts() {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.defaultPrevented) return;
      const s = shortcutOf(e);
      if (s && runShortcut(s)) e.preventDefault();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);
}
