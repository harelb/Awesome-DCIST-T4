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
  /** browser terminals on fleet machines (console --terminal) */
  terminal?: boolean;
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
  action_type?: string;
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

/** fleetlink C1 ``reloc``: what supports ``frame_align``. ROMAN loop closures
 *  between this robot and another robot id (the prior map), and how
 *  ``map -> odom`` moved. Ages are seconds on the robot clock at send. */
export interface RelocLoopClosure {
  prior: number;
  live_submap: number;
  prior_submap: number;
  assoc: number;
  age_s: number | null;
}

export interface RelocJump {
  dxy: number;
  /** degrees */
  dyaw: number;
  age_s: number | null;
}

export interface Reloc {
  prior_lcs: number;
  self_lcs?: number;
  best_assoc: number;
  last_lc_age_s: number | null;
  lcs: RelocLoopClosure[];
  jumps: number;
  last_jump: RelocJump | null;
  recent_jumps?: RelocJump[];
  total: {dxy: number; dyaw: number} | null;
}

/** fleetlink C1 ``phoenix_progress`` (phoenix only): distance from the pose
 *  to the end of the latest omniplanner Path; ``done`` within 1 m, once the
 *  robot has been farther than that from it since the plan arrived. */
export interface PhoenixProgress {
  plan_id: string;
  stamp?: number;
  dist_to_goal_m: number | null;
  done: boolean;
}

/** fleetlink C1 ``lidar``: an on-demand point cloud stream (off by default). */
export interface LidarState {
  /** the robot has a lidar (stays true when a cloud failed: see ``error``) */
  available: boolean;
  enabled: boolean;
  /** the agent's last lidar problem (bad cloud, no TF...), null when fine */
  error?: string | null;
  age_s?: number | null;
  bytes?: number;
}

export interface RobotState {
  /** "spot" | "phoenix" (agents before phoenix support: absent = spot) */
  platform?: string;
  phoenix_progress?: PhoenixProgress | null;
  lidar?: LidarState | null;
  pose?: Pose;
  frame_align?: FrameAlign;
  reloc?: Reloc;
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
  /** console-wide arrival counter (new refusals are toasted) */
  seq?: number;
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
  /** phoenix STOP latch: the console refuses motion until Resume or a fresh arm */
  stopped?: {t: number; reason: string} | null;
  caps?: {vx: number; vy: number; wz: number};
  deadman_s?: number;
  ros?: boolean;
}

/** plan repair v3 runtime constraint, e.g. {predicate: "forbidden-poi", symbols: ["o14"]} */
export interface ConstraintFact {
  predicate: "forbidden-poi" | "forbidden-edge" | string;
  symbols: string[];
}

/** goal_manager operator alert ("alert") or planner failure ("plan_failed") */
export interface RepairAlert {
  t: number;
  kind: "alert" | "plan_failed" | string;
  text: string;
  seq: number;
}

/** POST /api/robots/{r}/goal/{kind} answer (API.md §9). ``via``/``warning``/
 *  ``constraints`` only for PDDL goals. */
export interface GoalSendResult {
  ok: boolean;
  topic: string;
  msg: unknown;
  subscribers?: number;
  via?: "goal_manager" | "planner";
  warning?: string;
  constraints?: ConstraintFact[];
  plugin?: string;
}

export interface RobotPlan {
  goals: {t: number; kind: string; topic: string; msg: unknown; via?: "goal_manager" | "planner"; warning?: string}[];
  plan: null | {plan_id: string; robot_name: string; t: number; actions: PlanAction[]};
  results: ActionResult[];
  llm: string[];
  status?: PlanStatus[];
  /** plan repair v3 alerts (goal_manager + planner failures) */
  repair_alerts?: RepairAlert[];
  /** persistent constraints sent with every goal_manager goal */
  constraints?: ConstraintFact[];
  lease?: LeaseInfo | null;
}

/** GET /api/plan/repair */
export interface RepairStatus {
  available: boolean;
  goal_manager: boolean;
  subscribers: number;
  topic: string | null;
  error?: string | null;
}

export interface PlanRobot {
  robot: string;
  /** "spot" | "phoenix" (older consoles: absent = spot) */
  platform?: string;
  /** teleop hard caps for this robot's platform, when the backend reports them */
  caps?: {vx: number; vy: number; wz: number};
  plugins: {name: string; kind: "language" | "goto" | "tsp" | "pddl" | string; topic: string}[];
}

/** fleetlink budget classes; C6 is the image-store mirror. Unknown future classes
 *  still render (LivePanel lists whatever the link reports). */
export type Cls = "C1" | "C2" | "C3" | "C4" | "C5" | "C6" | (string & {});

export interface LinkInfo {
  connected: boolean;
  frame_align?: FrameAlign | null;
  reloc?: Reloc | null;
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
  /** mesh level of detail: the robot's (``lod`` null: an older robot sending
   *  full tiles), the one this console asks for, tiles still to arrive, bytes held */
  mesh?: MeshLodStatus;
  /** C6 image-store mirror switch: what the robot does (null: no store or an
   *  older agent without the switch) and what this console asked (null: nothing) */
  store_send?: {robot: boolean | null; wanted: boolean | null};
  /** base-side mirror of this robot's store (empty until a store is seen) */
  store?: {backlog?: number | null; frames?: number; watermark?: number | null} | null;
}

export interface MeshLodStatus {
  lod: "full" | "decimated" | null;
  lod_tiles?: Record<string, "full" | "decimated">;
  wanted: "full" | "decimated" | null;
  tiles: number;
  pending: number;
  bytes: number;
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
  /** network the robot <-> base links use; null = the topology default */
  link_network?: string | null;
  /** "router" | "fleetlink" for this launch; null = the topology default */
  link_mode?: string | null;
  /** run the pre-launch checks (default true); a NO-GO asks to confirm each issue */
  preflight?: boolean;
}

export type PreflightStatus = "go" | "warn" | "nogo" | "info";

/** One pre-launch check (fleet.preflight.Check). */
export interface PreflightCheck {
  machine: string;
  name: string;
  status: PreflightStatus;
  detail: string;
  hint?: string;
}

/** POST /api/launch/preflight */
export interface PreflightResult {
  checks: PreflightCheck[];
  verdicts: Record<string, PreflightStatus>;
  nogo: number;
  t: number;
}

export interface LaunchDefaults {
  robots?: string[];
  session?: string;
  base_station?: string | null;
  bs_session?: string;
  sessions?: string[];
  /** topology networks a launch can put the robot links on */
  link_networks?: string[];
  /** default link network (zenoh.connect_network, else the console's network) */
  link_network?: string | null;
  /** the network this console reaches the machines on */
  active_network?: string | null;
  /** robot link modes and the topology's default (fleet_defaults.fleet_link) */
  link_modes?: string[];
  link_mode?: string | null;
  /** base station that runs the fleetlink router */
  fleetlink_base?: string | null;
  [k: string]: unknown;
}
