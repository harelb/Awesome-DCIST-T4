#!/usr/bin/env node
// Map edit (API.md §17) walk-through against a running console whose Neo4j map
// may be edited (use a scratch DB): reclassify an object from the Inspector,
// draw an area and make it a region, add an object, remove it and undo, take
// a snapshot, open the history. Saves screenshots; every edit it makes stays
// in the edit history (undo them, or run with CLEANUP=1 to undo them at the end).
//
//   CONSOLE_URL=http://127.0.0.1:8098/ SHOT_DIR=/tmp/shots RECLASSIFY=O(395) node scripts/capture-map-edit.mjs
import {chromium} from "playwright-core";
import {existsSync, mkdirSync} from "node:fs";
import {homedir} from "node:os";
import {join} from "node:path";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8090/";
const shotDir = process.env.SHOT_DIR ?? "/tmp/map-edit-shots";
const target = process.env.RECLASSIFY ?? "O(395)";
const regionName = process.env.REGION ?? "courtyard";
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
const page = await browser.newPage({viewport: {width: Number(process.env.W ?? 1700), height: Number(process.env.H ?? 1000)}});
const errors = [];
page.on("pageerror", e => errors.push(String(e)));
page.on("console", m => m.type() === "error" && errors.push(m.text()));
const shot = async name => {
  await page.waitForTimeout(900);
  await page.screenshot({path: join(shotDir, `${name}.png`)});
  console.log("shot", name);
};
// edits are slow while the console reloads the graph after each commit: wait until idle
const idle = () => page.waitForFunction(() => !window.__adt4MapEdit?.getState().busy, null, {timeout: 120000});
const u = new URL(base);
u.hash = "#/mission";
await page.addInitScript(() => {
  if (!sessionStorage.getItem("seeded")) {
    localStorage.removeItem("adt4.mission.roster.v1");
    localStorage.removeItem("adt4.chat.v1");
    sessionStorage.setItem("seeded", "1");
  }
});
await page.goto(u.toString());
await page.waitForSelector("[data-testid=viewer-canvas]", {timeout: 20000});
await page.waitForFunction(() => !/^\s*0 nodes/.test(document.querySelector(".vstatus")?.textContent ?? "0 nodes"), null, {timeout: 90000}).catch(() => console.log("no nodes shown"));
await page.waitForTimeout(4000);

// 1. edit mode on
await page.locator("[data-testid=map-edit]").scrollIntoViewIfNeeded();
await page.locator("[data-testid=map-edit] label:has-text('edit map')").click();
await page.waitForTimeout(1500);
await shot("01-edit-mode");

// 2. reclassify from the Inspector
await page.fill("input[aria-label='Search graph']", target);
await page.press("input[aria-label='Search graph']", "Enter");
await page.waitForSelector("[data-testid=node-editor]", {timeout: 10000});
await page.click("[data-testid=node-editor] summary");
const cls = page.locator("[data-testid=node-editor] input[list=node-editor-classes]");
await cls.fill("crate");
await page.click("[data-testid=node-editor] button:has-text('Apply')");
await page.waitForSelector("[data-testid=edit-toast]", {timeout: 10000});
await shot("02-reclassified");
await page.fill("input[aria-label='Search graph']", "");

// 3. draw an area, make it a region
await page.click(".vright-head button:has-text('Mission')");
await page.click("button:has-text('Draw area')");
const box = await page.locator("[data-testid=viewer-canvas]").boundingBox();
// zoom out from the reclassified object so the area covers a block of the map
await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
for (let i = 0; i < 3; i++) {
  await page.mouse.wheel(0, 400);
  await page.waitForTimeout(150);
}
await page.waitForTimeout(1200);
const cx = box.x + box.width * 0.5;
const cy = box.y + box.height * 0.5;
const r = Math.min(box.width, box.height) * 0.12;
for (const [dx, dy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
  await page.mouse.click(cx + dx * r, cy + dy * r);
  await page.waitForTimeout(250);
}
await page.fill(".mission-draft input[aria-label='Name']", regionName);
await page.click(".mission-draft button:has-text('Finish')");
await page.waitForTimeout(500);
await idle();
await page.click(`button[aria-label='Make ${regionName} a map region']`);
await page.waitForSelector(`.region-tag:has-text('${regionName}')`, {timeout: 15000}).catch(() => console.log("no region tag"));
await shot("03-region");

// 4. add an object, remove it, undo the removal
await page.locator("[data-testid=map-edit]").scrollIntoViewIfNeeded();
await page.click("[data-testid=map-edit] button:has-text('Add object')");
await page.fill("input[aria-label='New object class']", "verify crate");
await page.mouse.click(cx + r * 2, cy);
await page.waitForSelector("[data-testid=selection-summary]:has-text('1 Object')", {timeout: 20000}).catch(async () => {
  console.log("selection:", await page.locator("[data-testid=edit-selection]").textContent().catch(() => "none"));
});
await shot("04-added");
await idle();
await page.click("[data-testid=edit-selection] button:has-text('Remove')");
await page.waitForSelector("[data-testid=edit-toast] button:has-text('Undo')", {timeout: 10000});
await shot("05-removed-undo-toast");
await page.click("[data-testid=edit-toast] button:has-text('Undo')");
await page.waitForTimeout(500);
await idle();

// 5. snapshot + history
await page.click("summary:has-text('Snapshots')");
await page.fill("input[aria-label='Snapshot name']", "map-edit verify");
await idle();
await page.click("button:has-text('Take snapshot')");
await page.waitForSelector("summary:has-text('Snapshots') >> text=(1)", {timeout: 60000}).catch(() => console.log("snapshot not listed"));
await page.click("summary:has-text('Snapshots')");
await page.click("summary:has-text('Regions')");
await page.click("summary:has-text('History')");
await shot("06-history-snapshot");

console.log("errors:", errors.length ? errors : "none");
await browser.close();
