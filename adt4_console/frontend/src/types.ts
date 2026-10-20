// Types for the ADT4 console REST/WS contract (adt4_console/docs/API.md).

export interface Health {
  ok: boolean;
  version: string;
  ros: boolean;
  ros_error?: string | null;
  fleetlink: boolean;
  fleetlink_error?: string | null;
  neo4j: Record<string, boolean>;
}

export interface ConsoleConfig {
  network: string;
  networks: string[];
  output_root: string;
  local_machine: string;
  neo4j_targets: {name: string; uri: string; prior: boolean}[];
  maps: {name: string; path: string}[];
  sessions: string[];
  fleet_defaults: Record<string, unknown>;
  robots: string[];
}

export interface Machine {
  name: string;
  role: string;
  ip: string;
  user: string;
  desc?: string;
  platform_id?: string;
  /** topology platform_type: spot / phoenix / ... */
  platform_type?: string;
  online?: boolean;
  ping?: boolean | number | null;
  ssh?: boolean | null;
  local?: boolean;
}

export interface FleetMachines {
  network: string;
  machines: Machine[];
}

export interface RemoteStatus {
  tmux_sessions?: string;
  ros2_procs?: string;
  load?: string;
  mem?: string;
  gpu?: string;
  disk?: string;
  battery?: string;
  radio?: string;
  [k: string]: unknown;
}

export interface FleetStatus {
  updated: number;
  machines: Record<string, RemoteStatus>;
}

export type NodeStatus = "NOMINAL" | "WARNING" | "ERROR" | "NO_HB" | "STARTUP";

export interface RobotNodes {
  readiness: [string, string] | {color: string; label: string} | string | null;
  nodes: {nickname: string; node_name: string; status: NodeStatus; notes: string; required?: boolean}[];
}

export interface FleetNodes {
  updated: number;
  robots: Record<string, RobotNodes>;
}

export type JobState = "running" | "waiting" | "succeeded" | "failed" | "cancelled";
export type LogLevel = "heading" | "info" | "dim" | "ok" | "warn" | "error";

export interface JobConfirm {
  kind: string;
  message: string;
  required_text?: string | null;
  yes_label?: string | null;
  no_label?: string | null;
  detail?: unknown;
}

export interface Job {
  id: string;
  kind: string;
  title: string;
  state: JobState;
  created: number;
  finished?: number | null;
  progress?: Record<string, number>;
  log?: {t: number; level: LogLevel; text: string}[];
  confirm?: JobConfirm | null;
  result?: unknown;
}

export interface ImageStoreInfo {
  /** frame counts by kind: keyframe / subkeyframe / object */
  frames: Partial<Record<"keyframe" | "subkeyframe" | "object", number>> & Record<string, number>;
  frames_total: number;
  observations: number;
  sensors: number;
  schema_version?: string | null;
  keyframes_unanchored?: number;
  /** du of <run>/image_store; null while sizes are pending */
  size_bytes?: number | null;
  check: {ok: boolean; problems: string[]};
}

export interface MapEntry {
  name: string;
  size: string | number;
  subdirs?: string[];
  is_prior?: boolean;
  path?: string;
  abs_path?: string;
  /** epoch seconds */
  mtime?: number | null;
  ctime?: number | null;
  name_date?: number | null;
  dsg_mtime?: number | null;
  size_bytes?: number | null;
  size_pending?: boolean;
  has_dsg?: boolean;
  has_dsg_mesh?: boolean;
  has_mesh?: boolean;
  has_image_store?: boolean;
  image_store?: ImageStoreInfo | null;
  sessions?: number;
  error?: string;
  /** run identity: run_info.json (launch), .adt4_origin.json (copy) or a fallback */
  identity?: RunIdentity | null;
  /** present on a transferred copy */
  origin?: RunOrigin | null;
  /** robot run: its copy on the base station */
  copy?: CopyLink | null;
  /** base copy: the robot run it copies */
  copy_of?: CopyOf | null;
}

export interface RunIdentity {
  run_id: string;
  source: "run_info" | "origin" | "fallback";
  experiment?: string;
  robot?: string | null;
  started_at?: string;
  host?: string;
}

export interface RunOrigin {
  run_id: string;
  source_machine: string;
  source_path: string;
  transferred_at: string;
  images?: string;
  compression?: string;
  robot?: string;
  experiment?: string;
}

export type CopyState = "in_sync" | "outdated" | "not_copied" | "unknown";

export interface CopyLink {
  state: CopyState;
  machine: string;
  name?: string;
  path?: string;
  matched_by?: "run_id" | "origin" | "contents";
  transferred_at?: string | null;
}

export interface CopyOf {
  machine: string;
  name: string;
  path: string;
  state: CopyState;
  matched_by?: string;
}

export interface MapsResponse {
  machines: Record<string, MapEntry[] | {error: string}>;
  /** machines whose sizes are still being measured in the background */
  pending?: string[];
  /** the machine copies are paired against (this host) */
  base?: string;
}

export interface TransferPlan {
  run_id: string;
  robot: string;
  dest_name: string;
  dest_path: string;
  status: "new" | "update" | "up_to_date" | "collision";
  state: CopyState;
  send_store: boolean;
  note: string;
}

export interface ActiveLaunch {
  active: {
    experiment: string;
    robots: string[];
    prior: null | {label: string; local_path: string | null; source_id: string | null; pending: boolean; source_machine: string; path: string; run_id: string | null};
  } | null;
  default_sources: string[];
}

export interface GraphSource {
  id: string;
  kind: "neo4j" | "file" | "live";
  label: string;
  version: number;
  nodes: number;
}

export interface TileManifest {
  tile_size: number;
  tiles: Record<string, {hash: string; nv: number; nf: number; origin: [number, number, number]}>;
}

export interface NodeDetail {
  sym: string;
  layer: number;
  part: number;
  label: number;
  label_name?: string;
  name?: string;
  pos: [number, number, number];
  bbox?: number[] | null;
  attributes?: Record<string, unknown>;
  images?: string[];
}

export interface Pose {
  x: number;
  y: number;
  z: number;
  qw: number;
  qx: number;
  qy: number;
  qz: number;
  frame?: string;
  t?: number;
}

export interface ActionResult {
  plan_id?: string;
  action_index?: number;
  status?: string;
  detail?: string;
  t?: number;
}

/** fleetlink C1 ``frame_align``: ``T`` (x, y, z, qw, qx, qy, qz) maps points in
 *  ``child`` (the robot's scene-graph frame, e.g. ``hamilton/odom``) into
 *  ``parent`` (the global ``map``). ``ok`` false = not relocalized (``T`` null). */
export interface FrameAlign {
  parent: string;
  child: string;
  T: number[] | null;
  /** TF stamp (robot clock); 0 = static transform */
  stamp?: number | null;
  ok: boolean;
  reason?: string;
  /** robot clock at send minus ``stamp`` */
  age_s?: number | null;
}

export interface RobotState {
  pose?: Pose;
  frame_align?: FrameAlign;
  guards?: Record<string, unknown>;
  health?: {nominal?: number; warning?: number; error?: number; nodes?: unknown};
  results?: ActionResult[];
  stats?: Record<string, unknown>;
}

export interface PlanAction {
  type: string;
  path?: [number, number, number][] | null;
  object_id?: string | null;
  object_class?: string | null;
  point?: [number, number, number] | null;
}

/** omniplanner ~/plan_status: one entry per handled goal. ``refusals`` names
 *  the robots the runtime dispatch guards refused ({robot: reason}). */
export interface PlanStatus {
  plugin: string;
  robot: string;
  ok: boolean;
  error: string;
  t: number;
  refusals?: Record<string, string>;
}

/** Spot lease from the executor's runtime guards (``t``: console seconds). */
export interface LeaseInfo {
  known: boolean;
  owned: boolean;
  owner: string;
  t?: number;
}

/** spot_executor_node ~/teleop_state (JSON), as relayed by the console. */
export interface TeleopState {
  active: boolean;
  allowed: boolean;
  interlock: string;
  busy: string;
  standing: boolean | null;
  commanded: {vx: number; vy: number; wz: number};
  caps: {vx: number; vy: number; wz: number};
  deadman_s: number;
  cmd_age_s: number | null;
  last_reject: string;
  last_event: string;
  commands: number;
  rejected: number;
  plan_executing: boolean;
  plan_paused: boolean;
  robot?: string;
  executor?: string;
  t?: number;
  rx_t?: number;
}

/** GET /api/robots/<r>/teleop and hub topic robot/<r>/teleop. */
export interface TeleopView {
  robot: string;
  state: TeleopState | null;
  lease?: LeaseInfo | null;
  link?: {connected?: boolean; rtt?: number | null; loss?: number | null; age_s?: number | null} | null;
  session?: {owner: string; moving: boolean; age_s: number; sent: number; deadman_stops: number} | null;
  caps?: {vx: number; vy: number; wz: number};
  deadman_s?: number;
  ros?: boolean;
}

export interface RobotPlan {
  goals: {t: number; kind: string; topic: string; msg: unknown}[];
  plan: null | {plan_id: string; robot_name: string; t: number; actions: PlanAction[]};
  results: ActionResult[];
  llm: string[];
  status?: PlanStatus[];
  lease?: LeaseInfo | null;
}

export interface PlanRobot {
  robot: string;
  plugins: {name: string; kind: "language" | "goto" | "tsp" | "pddl" | string; topic: string}[];
}

/** fleetlink budget classes; C6 is the image-store mirror. Unknown future classes
 *  still render (LivePanel lists whatever the link reports). */
export type Cls = "C1" | "C2" | "C3" | "C4" | "C5" | "C6" | (string & {});

export interface LinkInfo {
  connected: boolean;
  frame_align?: FrameAlign | null;
  epoch?: string;
  last_seen?: number | null;
  staleness_s?: number | null;
  latency_ms?: number | null;
  nodes?: number;
  tiles?: number;
  budgets?: Partial<Record<Cls, number>>;
  rates?: Partial<Record<Cls, number>>;
  drops?: number;
  gaps?: number;
  resyncs?: number;
  auto?: boolean;
}

export interface LiveResponse {
  robots: Record<string, LinkInfo>;
}

export interface Topology {
  networks?: Record<string, {subnet?: string}>;
  machines?: Record<
    string,
    {role: string; platform_id?: string; platform_type?: string; desc?: string; addresses?: Record<string, string>}
  >;
  [k: string]: unknown;
}

export interface LaunchPlan {
  robots: string[];
  session: string;
  experiment: string;
  mode: "mapping" | "relocalize";
  prior_path: string;
  prior_source: string | null;
  record: boolean;
  base_station: string | null;
  bs_session: string | null;
}

export interface LaunchDefaults {
  robots?: string[];
  session?: string;
  base_station?: string | null;
  bs_session?: string;
  sessions?: string[];
  [k: string]: unknown;
}
