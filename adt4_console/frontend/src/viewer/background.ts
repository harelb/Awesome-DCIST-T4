// The 3D view's effective background and colours derived from it (kept out of
// Scene.tsx so overlays can use them without an import cycle).
import {useMemo} from "react";
import {useViewer} from "../stores/viewer";
import {useViewStyle} from "../stores/viewStyle";
import {useTheme} from "../stores/theme";
import {useRobotColorOf} from "../lib/palette";
import {contrastFor, defaultBackground, readableOn} from "./style";

/** Effective background colour (user choice, else the style default for the console theme). */
export function useBackground(): string {
  const style = useViewer(s => s.vizStyle);
  const bg = useViewStyle(s => s.background);
  const theme = useTheme(s => s.theme);
  return bg ?? defaultBackground(style, theme);
}

/** Robot colour for 3D drawing: the robot's own colour, darkened only where it
 *  would wash out on a light background (dark backgrounds: unchanged). */
export function robotInk(hex: string, bg: string): string {
  return contrastFor(bg).light ? readableOn(hex, bg, 3) : hex;
}

export function useRobotColor3D(): (name: string) => string {
  const colorOf = useRobotColorOf();
  const bg = useBackground();
  return useMemo(() => (name: string) => robotInk(colorOf(name), bg), [colorOf, bg]);
}
