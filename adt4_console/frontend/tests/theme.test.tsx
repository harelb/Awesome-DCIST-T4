import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {applyTheme, DEFAULT_THEME, loadTheme, parseStoredTheme, saveTheme, THEME_STORAGE_KEY, useTheme} from "../src/stores/theme";
import {ThemeToggle} from "../src/components/Shell";
import {contrastFor, contrastRatio, defaultBackground, LIGHT_BACKGROUND, readableOn, STYLE_BACKGROUND} from "../src/viewer/style";
import {robotInk} from "../src/viewer/background";
import {pathColors} from "../src/viewer/LiveOverlays";
import {PALETTE_HEX, shade} from "../src/lib/palette";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const reset = () => {
  window.localStorage.clear();
  useTheme.setState({theme: "dark"});
  delete document.documentElement.dataset.theme;
  document.documentElement.style.colorScheme = "";
};

describe("theme persistence", () => {
  beforeEach(reset);
  afterEach(() => vi.restoreAllMocks());

  it("defaults to dark and only accepts known values", () => {
    expect(DEFAULT_THEME).toBe("dark");
    expect(parseStoredTheme(null)).toBe("dark");
    expect(parseStoredTheme("light")).toBe("light");
    expect(parseStoredTheme("dark")).toBe("dark");
    expect(parseStoredTheme("system")).toBe("dark");
    expect(parseStoredTheme("")).toBe("dark");
  });

  it("round-trips through localStorage", () => {
    saveTheme("light");
    expect(window.localStorage.getItem(THEME_STORAGE_KEY)).toBe("light");
    expect(loadTheme()).toBe("light");
  });

  it("survives blocked storage", () => {
    vi.spyOn(Storage.prototype, "getItem").mockImplementation(() => {
      throw new Error("blocked");
    });
    vi.spyOn(Storage.prototype, "setItem").mockImplementation(() => {
      throw new Error("blocked");
    });
    expect(loadTheme()).toBe("dark");
    expect(() => saveTheme("light")).not.toThrow();
    expect(() => useTheme.getState().setTheme("light")).not.toThrow();
    expect(useTheme.getState().theme).toBe("light");
  });

  it("applyTheme sets data-theme and color-scheme on the root", () => {
    applyTheme("light");
    expect(document.documentElement.dataset.theme).toBe("light");
    expect(document.documentElement.style.colorScheme).toBe("light");
    applyTheme("dark");
    expect(document.documentElement.dataset.theme).toBe("dark");
  });

  it("toggle flips, persists and applies", () => {
    useTheme.getState().toggle();
    expect(useTheme.getState().theme).toBe("light");
    expect(window.localStorage.getItem(THEME_STORAGE_KEY)).toBe("light");
    expect(document.documentElement.dataset.theme).toBe("light");
    useTheme.getState().toggle();
    expect(useTheme.getState().theme).toBe("dark");
    expect(window.localStorage.getItem(THEME_STORAGE_KEY)).toBe("dark");
    expect(document.documentElement.dataset.theme).toBe("dark");
  });
});

describe("ThemeToggle", () => {
  let host: HTMLDivElement;
  let root: Root;
  beforeEach(() => {
    reset();
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
  });
  afterEach(() => {
    act(() => root.unmount());
    host.remove();
  });

  it("labels the theme a click switches to and flips the app theme", () => {
    act(() => root.render(<ThemeToggle />));
    const btn = host.querySelector<HTMLButtonElement>("[data-testid=theme-toggle]")!;
    expect(btn.getAttribute("aria-label")).toBe("Switch to light theme");
    expect(btn.textContent).toContain("Light");
    act(() => btn.click());
    expect(useTheme.getState().theme).toBe("light");
    expect(document.documentElement.dataset.theme).toBe("light");
    expect(btn.getAttribute("aria-label")).toBe("Switch to dark theme");
    expect(btn.textContent).toContain("Dark");
  });
});

describe("3D colours under the light theme", () => {
  it("default background follows the theme; dark keeps the historical defaults", () => {
    expect(defaultBackground("hydra")).toBe(STYLE_BACKGROUND.hydra);
    expect(defaultBackground("classic", "dark")).toBe(STYLE_BACKGROUND.classic);
    expect(defaultBackground("hydra", "light")).toBe(LIGHT_BACKGROUND.hydra);
    for (const bg of Object.values(LIGHT_BACKGROUND)) {
      const ct = contrastFor(bg);
      expect(ct.light).toBe(true);
      expect(ct.edge).toBe("#000000");
      expect(contrastRatio(ct.text, bg)).toBeGreaterThan(7);
    }
  });

  it("readableOn darkens pale colours only as far as needed", () => {
    const bg = LIGHT_BACKGROUND.hydra;
    expect(readableOn("#000080", bg)).toBe("#000080"); // already fine
    for (const c of PALETTE_HEX) expect(contrastRatio(readableOn(c, bg, 3), bg)).toBeGreaterThanOrEqual(3);
  });

  it("robot colours are untouched on dark backgrounds", () => {
    for (const c of PALETTE_HEX) {
      expect(robotInk(c, STYLE_BACKGROUND.hydra)).toBe(c);
      expect(robotInk(c, STYLE_BACKGROUND.classic)).toBe(c);
      expect(pathColors(c, STYLE_BACKGROUND.hydra)).toEqual({smooth: shade(c, 1.45), mlp: c, target: shade(c, 0.6)});
    }
    expect(pathColors("#e6194b")).toEqual({smooth: shade("#e6194b", 1.45), mlp: "#e6194b", target: shade("#e6194b", 0.6)});
  });

  it("robot paths stay visible on the light background", () => {
    const bg = LIGHT_BACKGROUND.hydra;
    for (const c of PALETTE_HEX) {
      const p = pathColors(c, bg);
      expect(contrastRatio(p.mlp, bg)).toBeGreaterThanOrEqual(3);
      expect(contrastRatio(p.smooth, bg)).toBeGreaterThanOrEqual(2);
      expect(contrastRatio(robotInk(c, bg), bg)).toBeGreaterThanOrEqual(3);
    }
  });
});
