#!/usr/bin/env node
// Record an overhead video of a saved mission's simulation (#/mission, top view
// over the satellite ground) against a running console.
//
//   CONSOLE_URL=http://127.0.0.1:8099/ MISSION_ID=m1234abcd OUT_DIR=/tmp/m1 \
//     VIDEO_S=60 node scripts/record-mission.mjs
//
// Writes OUT_DIR/overhead.webm (Playwright's recorder; convert with ffmpeg),
// OUT_DIR/start.png and OUT_DIR/final.png. The sim speed is chosen so the whole
// mission plays in about VIDEO_S seconds (SPEED overrides it). No LLM calls.
import {chromium} from "playwright-core";
import {existsSync, mkdirSync, readdirSync, renameSync, rmSync, writeFileSync} from "node:fs";
import {homedir} from "node:os";
import {join} from "node:path";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8099/";
const id = process.env.MISSION_ID;
const outDir = process.env.OUT_DIR ?? "/tmp/mission-video";
const videoS = Number(process.env.VIDEO_S ?? 60);
const W = Number(process.env.W ?? 1600);
const H = Number(process.env.H ?? 1000);
if (!id) throw new Error("MISSION_ID is required");
mkdirSync(outDir, {recursive: true});
const api = (p, init) =>
  fetch(new URL(p, base), {headers: {"content-type": "application/json"}, ...init}).then(async r => {
    if (!r.ok) throw new Error(`${p}: ${r.status} ${await r.text()}`);
    return r.json();
  });

function findChrome() {
  if (process.env.CHROMIUM_PATH) return process.env.CHROMIUM_PATH;
  const c = join(homedir(), ".cache", "ms-playwright", "chromium-1234", "chrome-linux64", "chrome");
  return existsSync(c) ? c : chromium.executablePath();
}

const preview = await api(`api/missions/${id}/preview`);
const makespan = Number(preview.makespan ?? 0) || 120;
const speed = Number(process.env.SPEED ?? Math.max(1, Math.ceil(makespan / videoS)));
console.log(`mission ${id}: makespan ${makespan.toFixed(0)} s -> speed ${speed}x`);

const browser = await chromium.launch({
  executablePath: findChrome(),
  headless: true,
  args: ["--no-sandbox", "--disable-dev-shm-usage", "--enable-webgl", "--ignore-gpu-blocklist", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const vidTmp = join(outDir, ".video");
rmSync(vidTmp, {recursive: true, force: true});
const ctx = await browser.newContext({viewport: {width: W, height: H}, recordVideo: {dir: vidTmp, size: {width: W, height: H}}});
const page = await ctx.newPage();
const t0 = Date.now();
const errors = [];
page.on("pageerror", e => errors.push(String(e)));
const u = new URL(base);
u.hash = "#/mission";
await page.addInitScript(() => {
  if (!sessionStorage.getItem("seeded")) {
    localStorage.removeItem("adt4.chat.v1");
    sessionStorage.setItem("seeded", "1");
  }
});
await page.goto(u.toString());
await page.waitForSelector("[data-testid=viewer-canvas]", {timeout: 30000});
await page.waitForFunction(() => !/^\s*0 nodes/.test(document.querySelector(".vstatus")?.textContent ?? "0 nodes"), null, {timeout: 90000}).catch(() => console.log("no nodes shown"));
// overhead look: satellite ground only (no mesh), no node text labels (MESH=1 / TEXT=1 keep them)
await page.click("button:has-text('Layers')").catch(() => console.log("no Layers tab"));
await page.waitForTimeout(800);
if (process.env.MESH !== "1")
  for (const box of await page.locator("label.mini-check:has-text('mesh') input").all()) if (await box.isChecked()) await box.uncheck({force: true});
if (process.env.TEXT !== "1") {
  const t = page.locator("label:has-text('Text labels') input").first();
  if ((await t.count()) && (await t.isChecked())) await t.uncheck({force: true});
}
await page.click("button:has-text('Mission chat')").catch(() => undefined);
await page.selectOption("select[aria-label='Mission']", id);
await page.waitForTimeout(6000); // mesh / tiles / mission legs settle
await page.screenshot({path: join(outDir, "start.png")});
const sim = (cmd, value) => api(`api/missions/${id}/sim`, {method: "POST", body: JSON.stringify({cmd, value})});
await page.click("button:has-text('Fit')").catch(() => undefined);
await page.waitForTimeout(1500);
// the page's own transport: Simulate (no run yet) or Replay (a finished run) starts it and subscribes
const start = page.locator(".mission-transport button", {hasText: /Simulate|Replay|Play/}).first();
const simStartS = (Date.now() - t0) / 1000;
await start.click();
for (let i = 0; i < 20; i++) {  // the click's new+play land first, else they reset the speed
  await page.waitForTimeout(250);
  const r = await api(`api/missions/${id}/run`).catch(() => null);
  if (r && r.playing && !r.finished) break;
}
await sim("speed", speed);
const chk = await api(`api/missions/${id}/run`).catch(() => null);
console.log("run speed", chk?.speed, "t", chk?.t);
const deadline = Date.now() + (makespan / speed + 120) * 1000;
await page.waitForTimeout(2000);
for (;;) {
  const run = await api(`api/missions/${id}/run`).catch(() => null);
  if (run?.finished && run.t > 1) break;
  if (Date.now() > deadline) {
    console.log("sim did not finish in time");
    break;
  }
  await page.waitForTimeout(1000);
}
await page.waitForTimeout(3000);
await page.screenshot({path: join(outDir, "final.png")});
await page.close();
await ctx.close();
const vids = readdirSync(vidTmp).filter(f => f.endsWith(".webm"));
if (vids.length) renameSync(join(vidTmp, vids[0]), join(outDir, "overhead.webm"));
rmSync(vidTmp, {recursive: true, force: true});
writeFileSync(join(outDir, "video.json"), JSON.stringify({sim_start_s: simStartS, speed, makespan_s: makespan}, null, 1));
console.log("errors:", errors.length ? errors.slice(0, 5) : "none");
await browser.close();
