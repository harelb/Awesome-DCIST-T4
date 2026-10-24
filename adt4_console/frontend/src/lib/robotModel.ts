// Robot models for the 3D viewer: which baked GLB (public/models, made by
// scripts/bake_robot_models.py) draws a platform, and where its ground is
// relative to the frame the robot's pose is reported in.
import type {Platform} from "./platform";

export type RobotModelMode = "model" | "arrow";
export type RobotHighlight = "off" | "outline" | "tint";

export interface RobotModelSpec {
  url: string;
  /** z of the ground below the pose frame (Spot: body, ~0.5 m up; Husky: base_link) */
  groundZ: number;
  /** z of the top of the model, for the name tag */
  topZ: number;
  /** ground ring radius, about the model's footprint */
  ring: number;
}

const MODELS: Partial<Record<Platform, RobotModelSpec>> = {
  spot: {url: "models/spot.glb", groundZ: -0.51, topZ: 0.1, ring: 0.75},
  phoenix: {url: "models/husky.glb", groundZ: -0.13, topZ: 0.23, ring: 0.65},
};

/** The model for a platform, or null (no model: draw the arrow). */
export function robotModelFor(platform: Platform | string | null | undefined): RobotModelSpec | null {
  return (platform && MODELS[platform as Platform]) || null;
}

/** Outline shell thickness (m) and the tint strength used by the highlight. */
export const OUTLINE_M = 0.03;
export const TINT_INTENSITY = 0.45;

export const ROBOT_MODEL_MODES: {value: RobotModelMode; label: string}[] = [
  {value: "model", label: "Model"},
  {value: "arrow", label: "Arrow"},
];

export const ROBOT_HIGHLIGHTS: {value: RobotHighlight; label: string}[] = [
  {value: "outline", label: "Outline"},
  {value: "tint", label: "Tint"},
  {value: "off", label: "Off"},
];
