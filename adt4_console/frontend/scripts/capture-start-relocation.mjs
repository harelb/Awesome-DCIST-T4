#!/usr/bin/env node
// Moving mission agents' starts in the mission planner (#/mission), against a
// running console (or vite dev in front of one): opens MISSION, types a start in
// the roster, drags another agent on the map (Esc-cancel first, then a real
// move), checks each move made a revision whose previewed path starts at the
// new start, and saves screenshots. Desktop (mouse) and Pixel 7 (touch, CDP
// touch events) passes.
//
//   CONSOLE_URL=http://127.0.0.1:5199/ MISSION=m1234abcd SHOT_DIR=/tmp/shots node scripts/capture-start-relocation.mjs
//
// MISSION needs ground agents spot1 + spot2 with a task each (t1: spot1, t2: spot2).
// It changes the mission's starts: run it on a scratch console, not the field one.
import {chromium, devices} from "playwright-core";
import {existsSync, mkdirSync} from "node:fs";
import {homedir} from "node:os";
import {join} from "node:path";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:5199/";
const mission = process.env.MISSION;
if (!mission) throw new Error("MISSION=<id> is required");
const shotDir = process.env.SHOT_DIR ?? "/tmp/start-shots";
mkdirSync(shotDir, {recursive: true});

function findChrome() {
  if (process.env.CHROMIUM_PATH) return process.env.CHROMIUM_PATH;
  const c = join(homedir(), ".cache", "ms-playwright", "chromium-1234", "chrome-linux64", "chrome");
  return existsSync(c) ? c : chromium.executablePath();
}

const browser = await chromium.launch({
  executablePath: findChrome(),
  headless: true,
  args: ["--no-sandbox", "--disable-dev-shm-usage", "--enable-webgl", "--ignore-gpu-blocklist", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const fails = [];
const check = (ok, what) => {
  console.log(ok ? "ok  " : "FAIL", what);
  if (!ok) fails.push(what);
};

const state = page =>
  page.evaluate(() => {
    const s = window.__adt4Mission.getState();
    return {
      rev: s.current?.revision ?? null,
      starts: Object.fromEntries((s.current?.agents ?? []).map(a => [a.id, a.start])),
      legs: Object.fromEntries(Object.entries(s.preview?.tasks ?? {}).map(([k, t]) => [k, t.path[0]])),
      error: s.error,
      roster: s.roster.map(a => a.id),
    };
  });
const waitRev = (page, rev) => page.waitForFunction(r => (window.__adt4Mission.getState().current?.revision ?? 0) > r && !window.__adt4Mission.getState().busy, rev, {timeout: 15000});
const screenOf = (page, id) => page.evaluate(i => window.__adt4AgentScreen?.(i) ?? null, id);

async function open(page, tag) {
  const u = new URL(base);
  u.hash = "#/mission";
  await page.addInitScript(() => {
    if (!sessionStorage.getItem("seeded")) {
      localStorage.setItem("adt4.mission.roster.v1", JSON.stringify({roster: [], areas: [], planted: [], fences: []}));
      sessionStorage.setItem("seeded", "1");
    }
  });
  await page.goto(u.toString());
  await page.waitForSelector("[data-testid=viewer-canvas]", {timeout: 20000});
  await page.waitForFunction(() => !/^\s*0 nodes/.test(document.querySelector(".vstatus")?.textContent ?? "0 nodes"), null, {timeout: 60000}).catch(() => console.log(tag, "no nodes shown"));
  await page.waitForTimeout(2500);
}

const shot = async (page, name) => {
  await page.waitForTimeout(700);
  await page.screenshot({path: join(shotDir, `${name}.png`)});
  console.log("shot", name);
};

async function pickMission(page) {
  await page.selectOption("select[aria-label='Mission']", mission);
  await page.waitForFunction(m => window.__adt4Mission.getState().current?.id === m, mission, {timeout: 15000});
  await page.waitForTimeout(800);
}

/** zoom the camera in around a map agent (mouse wheel dollies to the cursor) */
async function zoomTo(page, id, ticks) {
  for (let i = 0; i < ticks; i++) {
    const p = await screenOf(page, id);
    if (!p) return;
    await page.mouse.move(p[0], p[1]);
    await page.mouse.wheel(0, -300);
    await page.waitForTimeout(150);
  }
  await page.waitForTimeout(600);
}

// ------------------------------------------------------------------ desktop
{
  const page = await browser.newPage({viewport: {width: 1600, height: 1000}});
  const errors = [];
  page.on("pageerror", e => errors.push(String(e)));
  page.on("console", m => m.type() === "error" && errors.push(m.text()));
  await open(page, "desktop");
  await pickMission(page);
  let s = await state(page);
  check(s.roster.includes("spot1") && s.roster.includes("spot2"), `roster shows the mission's agents (${s.roster})`);
  await zoomTo(page, "spot1", Number(process.env.ZOOM ?? 0));
  await page.locator("[data-testid=start-edit-spot1]").scrollIntoViewIfNeeded();
  await shot(page, "d01-mission-open");

  // numeric entry
  const s0 = s.starts.spot1;
  const x = s0[0] + 8;
  const y = s0[1] + 6;
  const rev0 = s.rev;
  await page.fill("input[aria-label='spot1 start map x (m)']", x.toFixed(2));
  await page.fill("input[aria-label='spot1 start map y (m)']", y.toFixed(2));
  await page.press("input[aria-label='spot1 start map y (m)']", "Enter");
  await waitRev(page, rev0);
  s = await state(page);
  check(Math.hypot(s.starts.spot1[0] - x, s.starts.spot1[1] - y) < 0.011, `typed start reached the mission ${JSON.stringify(s.starts.spot1)}`);
  check(s.legs.t1 && Math.hypot(s.legs.t1[0] - x, s.legs.t1[1] - y) < 0.05, `re-preview: t1 starts at the typed start ${JSON.stringify(s.legs.t1)}`);
  await shot(page, "d02-typed-start");

  // a bad number: shown, not sent
  await page.fill("input[aria-label='spot1 start map x (m)']", "abc");
  await page.press("input[aria-label='spot1 start map x (m)']", "Enter");
  check((await page.locator("[data-testid=start-edit-spot1] [role=alert]").textContent())?.includes("x must be a number"), "invalid x is flagged");
  await shot(page, "d03-invalid");
  await page.press("input[aria-label='spot1 start map x (m)']", "Escape");
  check((await state(page)).rev === s.rev, "invalid entry made no revision");

  // drag spot2 then Esc: nothing changes
  s = await state(page);
  let p = await screenOf(page, "spot2");
  check(!!p, `spot2 on screen at ${JSON.stringify(p)}`);
  const cam0 = await page.evaluate(() => document.querySelector("[data-testid=viewer-canvas] canvas")?.toDataURL?.().length ?? 0);
  await page.mouse.move(p[0], p[1]);
  await page.mouse.down();
  for (let i = 1; i <= 10; i++) await page.mouse.move(p[0] + 12 * i, p[1] - 6 * i);
  await page.waitForTimeout(300);
  const ghost = await page.locator("[data-testid=start-drag-readout]").textContent().catch(() => null);
  check(!!ghost && ghost.includes("spot2"), `drag readout: ${ghost}`);
  await shot(page, "d04-dragging-then-esc");
  await page.keyboard.press("Escape");
  await page.mouse.up();
  await page.waitForTimeout(800);
  check((await state(page)).rev === s.rev, "Esc cancelled the drag (no revision)");
  const p1 = await screenOf(page, "spot2");
  check(Math.hypot(p1[0] - p[0], p1[1] - p[1]) < 2, "the camera did not move while dragging the glyph");
  void cam0;

  // drag spot2 for real
  p = p1;
  await page.mouse.move(p[0], p[1]);
  await page.mouse.down();
  for (let i = 1; i <= 12; i++) await page.mouse.move(p[0] - 10 * i, p[1] + 8 * i);
  await page.waitForTimeout(300);
  await shot(page, "d05-dragging");
  const before = s.starts.spot2;
  await page.mouse.up();
  await waitRev(page, s.rev);
  s = await state(page);
  check(Math.hypot(s.starts.spot2[0] - before[0], s.starts.spot2[1] - before[1]) > 1, `dragged start reached the mission ${JSON.stringify(before)} -> ${JSON.stringify(s.starts.spot2)}`);
  check(s.legs.t2 && Math.hypot(s.legs.t2[0] - s.starts.spot2[0], s.legs.t2[1] - s.starts.spot2[1]) < 0.05, `re-preview: t2 starts at the dragged start ${JSON.stringify(s.legs.t2)}`);
  await shot(page, "d06-dragged");

  // undo
  await page.click("button:has-text('Undo: move')");
  await waitRev(page, s.rev);
  const u = await state(page);
  check(Math.hypot(u.starts.spot2[0] - before[0], u.starts.spot2[1] - before[1]) < 0.011, `undo put spot2 back ${JSON.stringify(u.starts.spot2)}`);

  // lat/lon view of the starts
  const latlon = page.getByRole("tab", {name: "lat/lon"});
  if (await latlon.count()) {
    await latlon.click();
    await page.waitForTimeout(300);
    await page.locator("[data-testid=start-edit-spot1]").scrollIntoViewIfNeeded();
    await shot(page, "d07-latlon");
    await page.getByRole("tab", {name: "map x/y"}).click();
  } else console.log("no GPS anchor: no lat/lon");
  console.log("desktop errors:", errors.length ? errors : "none");
  await page.close();
}

// ------------------------------------------------------------------ phone
{
  const ctx = await browser.newContext({...devices["Pixel 7"]});
  const page = await ctx.newPage();
  const errors = [];
  page.on("pageerror", e => errors.push(String(e)));
  await open(page, "phone");
  // the mission panel is the right sheet
  await page.click("button[aria-label='Toggle side panel']");
  await page.waitForTimeout(600);
  await pickMission(page);
  await page.locator("[data-testid=start-edit-spot1]").scrollIntoViewIfNeeded();
  await shot(page, "p01-roster");
  const overflow = await page.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth);
  check(overflow <= 0, `no horizontal scroll (overflow ${overflow}px)`);
  const sizes = await page.evaluate(() =>
    [...document.querySelectorAll(".start-edit input, .start-edit button")].map(e => {
      const r = e.getBoundingClientRect();
      return {h: r.height, w: r.width, f: parseFloat(getComputedStyle(e).fontSize), t: e.tagName, type: e.getAttribute("type")};
    }),
  );
  const small = sizes.filter(z => z.type !== "checkbox" && (z.h < 40 || (z.t === "INPUT" && z.f < 16)));
  check(small.length === 0, `start controls ≥ 40 px, inputs ≥ 16 px font (${sizes.length} checked${small.length ? `, small: ${JSON.stringify(small)}` : ""})`);

  // numeric entry on the phone
  let s = await state(page);
  const x = s.starts.spot1[0] - 5;
  await page.fill("input[aria-label='spot1 start map x (m)']", x.toFixed(2));
  await page.press("input[aria-label='spot1 start map x (m)']", "Enter");
  await waitRev(page, s.rev);
  s = await state(page);
  check(Math.abs(s.starts.spot1[0] - x) < 0.011, `phone: typed x reached the mission ${JSON.stringify(s.starts.spot1)}`);

  // close the sheet, touch-drag spot2
  await page.click("button[aria-label='Toggle side panel']");
  await page.waitForTimeout(800);
  await zoomTo(page, "spot2", 0);
  const p = await screenOf(page, "spot2");
  check(!!p, `phone: spot2 on screen ${JSON.stringify(p)}`);
  const cdp = await ctx.newCDPSession(page);
  const touch = (type, pts) => cdp.send("Input.dispatchTouchEvent", {type, touchPoints: pts.map(([px, py]) => ({x: px, y: py, id: 1, radiusX: 4, radiusY: 4, force: 1}))});
  await touch("touchStart", [[p[0], p[1]]]);
  for (let i = 1; i <= 12; i++) {
    await touch("touchMove", [[p[0] + 6 * i, p[1] - 9 * i]]);
    await page.waitForTimeout(16);
  }
  await page.waitForTimeout(300);
  await shot(page, "p02-touch-dragging");
  const before = s.starts.spot2;
  await touch("touchEnd", []);
  await waitRev(page, s.rev).catch(() => undefined);
  s = await state(page);
  check(Math.hypot(s.starts.spot2[0] - before[0], s.starts.spot2[1] - before[1]) > 1, `phone: touch drag moved spot2 ${JSON.stringify(before)} -> ${JSON.stringify(s.starts.spot2)}`);
  check(s.legs.t2 && Math.hypot(s.legs.t2[0] - s.starts.spot2[0], s.legs.t2[1] - s.starts.spot2[1]) < 0.05, `phone: re-preview from the dragged start ${JSON.stringify(s.legs.t2)}`);
  await shot(page, "p03-touch-dragged");
  console.log("phone errors:", errors.length ? errors : "none");
  await ctx.close();
}

await browser.close();
console.log(fails.length ? `${fails.length} FAILED` : "all checks passed");
process.exit(fails.length ? 1 : 0);
