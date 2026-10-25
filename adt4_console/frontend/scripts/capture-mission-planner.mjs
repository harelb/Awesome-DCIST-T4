#!/usr/bin/env node
// End-to-end capture of the mission planner (#/mission) against a running console:
// seeds the roster / area / mock object, asks the chat for a mission, simulates
// it, accepts the quad's detection and saves screenshots along the way.
//
//   CONSOLE_URL=http://127.0.0.1:8097/ SHOT_DIR=/tmp/shots node scripts/capture-mission-planner.mjs
//
// The chat turn uses the console's LLM (a few cents). MISSION_TEXT overrides the command.
import {chromium} from "playwright-core";
import {existsSync, mkdirSync} from "node:fs";
import {homedir} from "node:os";
import {join} from "node:path";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8090/";
const shotDir = process.env.SHOT_DIR ?? "/tmp/mission-shots";
mkdirSync(shotDir, {recursive: true});
const text =
  process.env.MISSION_TEXT ??
  "quad1: search area A1 for a crate. When it is found, spot1 inspects it. Meanwhile spot2 goes to the nearest trash can and then returns home.";

function findChrome() {
  if (process.env.CHROMIUM_PATH) return process.env.CHROMIUM_PATH;
  const c = join(homedir(), ".cache", "ms-playwright", "chromium-1234", "chrome-linux64", "chrome");
  return existsSync(c) ? c : chromium.executablePath();
}

const ROSTER = {
  roster: [
    {id: "quad1", kind: "quad", start: [-45, -69], binding: null},
    {id: "spot1", kind: "spot", start: [-43, -71], binding: null},
    {id: "spot2", kind: "spot", start: [-41, -67], binding: null},
  ],
  areas: [{name: "A1", polygon: [[10, -60], [50, -60], [50, -100], [10, -100]]}],
  planted: [{label: "crate", pos: [30, -80], size_m: 1.5}],
  fences: [],
};

const browser = await chromium.launch({
  executablePath: findChrome(),
  headless: true,
  args: ["--no-sandbox", "--disable-dev-shm-usage", "--enable-webgl", "--ignore-gpu-blocklist", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const page = await browser.newPage({viewport: {width: Number(process.env.W ?? 1700), height: Number(process.env.H ?? 1000)}});
const errors = [];
page.on("pageerror", e => errors.push(String(e)));
page.on("console", m => m.type() === "error" && errors.push(m.text()));
const shot = async name => {
  await page.waitForTimeout(800);
  await page.screenshot({path: join(shotDir, `${name}.png`)});
  console.log("shot", name);
};

const wsGot = {};
if (process.env.WS_DEBUG === "1")
  page.on("websocket", w =>
    w.on("framereceived", f => {
      const m = /topic.(mission\/[a-z0-9]+)/.exec(Buffer.from(f.payload).toString("latin1"));
      if (m) wsGot[m[1]] = (wsGot[m[1]] ?? 0) + 1;
    }),
  );
const u = new URL(base);
u.hash = "#/mission";
await page.addInitScript(r => {
  if (!sessionStorage.getItem("seeded")) {
    localStorage.setItem("adt4.mission.roster.v1", JSON.stringify(r));
    localStorage.removeItem("adt4.chat.v1");
    sessionStorage.setItem("seeded", "1");
  }
}, ROSTER);
await page.goto(u.toString());
await page.waitForSelector("[data-testid=viewer-canvas]", {timeout: 20000});
await page.waitForFunction(() => !/^\s*0 nodes/.test(document.querySelector(".vstatus")?.textContent ?? "0 nodes"), null, {timeout: 60000}).catch(() => console.log("no nodes shown"));
await page.waitForTimeout(4000);
await shot("01-mission-page");

if (process.env.SKIP_CHAT !== "1") {
  await page.fill("textarea[aria-label='Chat message']", text);
  await page.click("button:has-text('Send')");
  await page.waitForSelector(".mission-card", {timeout: 180000});
  await page.waitForTimeout(1500);
  await shot("02-chat-mission-card");
  await page.click(".mission-card button:has-text('Simulate')");
  await page.waitForTimeout(1500);
  await page.selectOption("select[aria-label='Speed']", "10").catch(() => undefined);
  await page.waitForTimeout(4000);
  await shot("03-simulating");
  const cand = await page.waitForSelector(".mission-cand.pending", {timeout: 60000}).catch(() => null);
  if (cand) {
    await shot("04-detection");
    await page.click(".mission-cand.pending .thumb");
    await shot("05-quad-image");
    await page.click(".mission-imgview button:has-text('Close')");
    await page.click(".mission-cand.pending button:has-text('Accept')");
  } else console.log("no detection appeared");
  await page.waitForFunction(() => document.querySelector(".mission-transport")?.textContent?.includes("✓"), null, {timeout: 60000}).catch(() => console.log("sim did not finish"));
  await shot("06-finished");
}
if (process.env.WS_DEBUG === "1") console.log("mission frames", wsGot, await page.locator(".mission-transport").textContent());
console.log("errors:", errors.length ? errors : "none");
await browser.close();
