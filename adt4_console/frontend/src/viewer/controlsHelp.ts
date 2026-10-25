// Operator controls legend for the robot viewer: the single place to update when
// a control changes (Scene.tsx camera bindings, picking, Panes.tsx windows).
export type HelpMode = "3d" | "top";
export interface HelpEntry {
  /** the input, e.g. "Left-drag" or a key */
  keys: string;
  /** what it does */
  does: string;
}
export interface HelpSection {
  title: string;
  entries: HelpEntry[];
}

/** Camera controls per view mode (Scene.tsx CameraControls bindings). */
export const MODE_HELP: Record<HelpMode, HelpSection> = {
  "3d": {
    title: "3D view",
    entries: [
      {keys: "Left-drag", does: "Orbit"},
      {keys: "Right-drag", does: "Pan"},
      {keys: "Wheel / pinch", does: "Zoom"},
      {keys: "One-finger drag", does: "Orbit (touch)"},
      {keys: "Two-finger drag", does: "Pan + zoom (touch)"},
    ],
  },
  top: {
    title: "Top view",
    entries: [
      {keys: "Left-drag", does: "Pan"},
      {keys: "Right-drag", does: "Rotate the map in plane"},
      {keys: "Wheel / pinch", does: "Zoom"},
      {keys: "Two-finger twist", does: "Rotate (touch)"},
      {keys: "Q / E", does: "Rotate 90° left / right"},
      {keys: "R", does: "North up"},
    ],
  },
};

/** Picking and other controls that work in both modes. */
export const COMMON_HELP: HelpSection[] = [
  {
    title: "Picking",
    entries: [
      {keys: "Click node", does: "Select it (Inspect tab)"},
      {keys: "Click bounding box", does: "Select that object"},
      {keys: "Click empty space", does: "Clear the selection"},
      {keys: "Search result", does: "Select + fly to the node"},
      {keys: "Fly to (Inspect)", does: "Fly the camera to the selection"},
      {keys: "Goto picking", does: "Click places, objects or the mesh to add goto points"},
    ],
  },
  {
    title: "Camera tab: All cameras video",
    entries: [
      {keys: "Mosaic / Per camera", does: "One tiled robot stream (default) / one stream per camera"},
      {keys: "low / medium", does: "Grid quality (tile size, fps, total bitrate)"},
      {keys: "Camera chips", does: "Switch cameras on/off · All / None (saved per robot)"},
      {keys: "Drag chip / ‹ ›", does: "Reorder the tiles (= mosaic order)"},
      {keys: "Click tile", does: "Solo that camera, larger · click again or Back to all"},
      {keys: "full", does: "Open that camera's full live video"},
    ],
  },
  {
    title: "Teleop tab: drive the focused robot",
    entries: [
      {keys: "Drive", does: "Arm driving (asks once per session; red ARMED banner)"},
      {keys: "W / S", does: "Forward / back (panel focused and armed)"},
      {keys: "A / D", does: "Strafe left / right (Spot)"},
      {keys: "Q / E", does: "Turn left / right"},
      {keys: "Shift", does: "Slow (30 % of the speed slider)"},
      {keys: "Space", does: "Stop driving (zero velocity)"},
      {keys: "Joystick", does: "Touch drive: up = forward, sideways = turn (or strafe, Spot)"},
      {keys: "Click outside / switch tab / change focus", does: "Disarms; the robot stops within 0.5 s"},
      {keys: "Stop", does: "Halt the robot, end teleop and cancel the plan (Husky: also manual mode)"},
      {keys: "Stand / Sit · Pause / Resume", does: "Spot: posture · pause the plan to teleop, resume hands back"},
      {keys: "Resume (auto)", does: "Husky: leave manual mode, plans execute again"},
    ],
  },
  {
    title: "Robot focus",
    entries: [
      {keys: "Focus picker (toolbar)", does: "All robots or one robot: camera, teleop, plan, live link and the map follow it"},
      {keys: "[ / ]", does: "Previous / next robot (All robots is in the cycle)"},
      {keys: "Legend chip · camera tile", does: "Focus that robot (focused chip: back to all)"},
      {keys: "Drive layout", does: "Map + the focused robot's camera + Teleop at once (on by default while a robot is focused)"},
      {keys: "Follow", does: "Keep the 3D camera on the focused robot"},
    ],
  },
  {
    title: "Keys",
    entries: [
      {keys: "Enter (search)", does: "Pick the first hit"},
      {keys: "P", does: "Save a screenshot of the 3D view"},
      {keys: "Esc", does: "Clear search (in the box) · close this card / dialogs"},
      {keys: "/", does: "Focus search (Maps page)"},
    ],
  },
];

/** Mission planner and map editing (ToolBanner.tsx drawing keys, shortcuts.ts,
 *  MissionOverlay.tsx start dragging). None of the keys fire while typing. */
export const MISSION_HELP: HelpSection = {
  title: "Mission / map edit",
  entries: [
    {keys: "Enter / double-click", does: "Finish the area or lasso being drawn"},
    {keys: "Backspace", does: "Drop the drawing's last corner"},
    {keys: "Esc", does: "Cancel the armed tool / the start being dragged"},
    {keys: "Drag an agent", does: "Move its start (Shift inverts snap to places)"},
    {keys: "Shift-click", does: "Add to / remove from the edit selection"},
    {keys: "Delete", does: "Remove the edit selection (undoable)"},
    {keys: "Ctrl / ⌘ + Z", does: "Undo the newest of: your last start move, your last map edit (this browser)"},
    {keys: "F", does: "Fit the open mission in view"},
    {keys: "M", does: "Full map: the map alone, full screen (M / Esc to leave)"},
    {keys: "Space", does: "Simulate · pause / resume the simulation"},
  ],
};

/** Pane layout controls (wide screens only; Panes.tsx). */
export const LAYOUT_HELP: HelpSection = {
  title: "Layout",
  entries: [
    {keys: "Drag pane edge", does: "Resize panel · double-click resets"},
    {keys: "⧉ / drag a tab out", does: "Pop a side-panel tab into a window"},
    {keys: "Drag title bar", does: "Move window (snaps to edges)"},
    {keys: "Drag window edge", does: "Resize window"},
    {keys: "⇥ / drop on tab strip", does: "Dock the window again"},
    {keys: "– / ×", does: "Minimize / close window"},
    {keys: "Reset layout", does: "Defaults (Layout section, left panel)"},
  ],
};

/** One-time hint shown the first time the operator switches to Top. */
export const TOP_HINT = "Right-drag to rotate · Q/E 90° · R north up";
/** the same hint for touch screens (no right button, no keys) */
export const TOP_HINT_TOUCH = "One finger pans · two fingers pinch to zoom, drag to rotate";
export const TOP_HINT_KEY = "adt4.viewer.topHintDismissed";

/** Sections for the help card in the given mode (the mission planner lists its
 *  own keys first). */
export function helpSections(mode: HelpMode, withLayout: boolean, mission = false): HelpSection[] {
  const common = mission ? [MISSION_HELP, ...COMMON_HELP] : [...COMMON_HELP, MISSION_HELP];
  return [MODE_HELP[mode], ...common, ...(withLayout ? [LAYOUT_HELP] : [])];
}

export function hintDismissed(storage: Pick<Storage, "getItem"> | null | undefined): boolean {
  try {
    return storage?.getItem(TOP_HINT_KEY) === "1";
  } catch {
    return false;
  }
}

export function dismissHint(storage: Pick<Storage, "setItem"> | null | undefined) {
  try {
    storage?.setItem(TOP_HINT_KEY, "1");
  } catch {
    /* storage unavailable: the hint just comes back next visit */
  }
}
