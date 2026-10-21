// Screenshot tour of the phone full-screen drive view against the mock server,
// with real (CDP) multi-touch on the two sticks:
//
//   CONSOLE_URL=http://127.0.0.1:8291/ GRID_URL=http://127.0.0.1:8292/ SHOT_DIR=/tmp/shots node scripts/capture-drive-full.mjs
//
// CONSOLE_URL: a mock with MOCK_VIDEO (live video on euclid); GRID_URL: a mock
// without it (euclid's six cameras as stills). Prints what the robot was sent.
import {mkdirSync} from "node:fs";
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8291/";
const gridBase = process.env.GRID_URL ?? "http://127.0.0.1:8292/";
const robot = process.env.ROBOT ?? "euclid";
const shotDir = process.env.SHOT_DIR ?? "shots/drive_full";
const chrome = process.env.CHROMIUM_PATH ?? join(process.env.HOME, ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome");
mkdirSync(shotDir, {recursive: true});
const browser = await chromium.launch({executablePath: chrome, headless: true, args: ["--no-sandbox", "--use-angle=vulkan", "--enable-features=Vulkan", "--ignore-gpu-blocklist"]});
const errors = [];

async function phone(width, height, url) {
  const ctx = await browser.newContext({viewport: {width, height}, deviceScaleFactor: 2, isMobile: true, hasTouch: true});
  await ctx.addInitScript(r => localStorage.setItem("adt4.viewer.focus.v1", r), robot);
  const page = await ctx.newPage();
  page.on("pageerror", e => errors.push(String(e)));
  const cdp = await ctx.newCDPSession(page);
  await page.goto(`${url}#/viewer`);
  await page.locator("canvas").first().waitFor({timeout: 60_000});
  await page.waitForTimeout(1200);
  return {page, cdp, url};
}
const state = async ({page, url}) => (await page.evaluate(async u => (await fetch(u)).json(), `${url}api/robots/${robot}/teleop`)).state;
const touch = (cdp, type, pts) => cdp.send("Input.dispatchTouchEvent", {type, touchPoints: pts.map(p => ({x: p[0], y: p[1], id: p[2]}))});
const shot = (page, name) => page.screenshot({path: join(shotDir, `drive_full_${name}.png`)});

async function open(p) {
  await p.page.locator('[data-testid="drive-full-open"]').first().tap();
  await p.page.locator('[data-testid="drive-full"]').waitFor();
}

// --- portrait, live video ---------------------------------------------------
const p = await phone(390, 844, base);
await shot(p.page, "0_entry_button");
await open(p);
await p.page.waitForTimeout(2500);
await shot(p.page, "1_portrait_idle");
await p.page.locator('[data-testid="drive-full-arm"]').tap();
await p.page.locator('[data-testid="drive-full-arm-yes"]').waitFor();
await shot(p.page, "2_confirm");
await p.page.locator('[data-testid="drive-full-arm-yes"]').tap();
await p.page.waitForTimeout(300);

// two fingers: left forward + slightly right, right turning left
const L = [90, 620];
const R = [300, 620];
await touch(p.cdp, "touchStart", [[...L, 1]]);
await touch(p.cdp, "touchStart", [[...L, 1], [...R, 2]]);
for (let i = 1; i <= 8; i++) {
  await touch(p.cdp, "touchMove", [[L[0] + i * 3, L[1] - i * 10, 1], [R[0] - i * 8, R[1], 2]]);
  await p.page.waitForTimeout(40);
}
await p.page.waitForTimeout(400);
console.log("two sticks     ", JSON.stringify((await state(p))?.commanded));
await shot(p.page, "3_two_sticks");
await touch(p.cdp, "touchEnd", [[L[0] + 24, L[1] - 80, 1]]); // CDP touchEnd lists the lifted points: the left finger only
await p.page.waitForTimeout(300);
console.log("left lifted    ", JSON.stringify((await state(p))?.commanded));
await touch(p.cdp, "touchEnd", []);
await p.page.waitForTimeout(300);
console.log("both lifted    ", JSON.stringify((await state(p))?.commanded));

await p.page.locator('[data-testid="drive-full-view"]').tap();
await p.page.waitForTimeout(200);
await shot(p.page, "4_view_menu");
await p.page.getByRole("menuitemradio", {name: "Map"}).tap();
await p.page.waitForTimeout(800);
await shot(p.page, "5_map_view");
await p.page.locator('[data-testid="drive-full-view"]').tap();
await p.page.getByRole("menuitemradio", {name: "rgb"}).tap();

// page hidden -> disarmed
await touch(p.cdp, "touchStart", [[...L, 1]]);
await touch(p.cdp, "touchMove", [[L[0], L[1] - 80, 1]]);
await p.page.waitForTimeout(300);
console.log("driving again  ", JSON.stringify((await state(p))?.commanded));
await p.page.evaluate(() => {
  Object.defineProperty(document, "visibilityState", {value: "hidden", configurable: true});
  document.dispatchEvent(new Event("visibilitychange"));
});
await p.page.waitForTimeout(400);
console.log("page hidden    ", JSON.stringify((await state(p))?.commanded), "arm:", await p.page.locator('[data-testid="drive-full-arm"]').textContent());
await touch(p.cdp, "touchEnd", []);
await p.page.context().close();

// --- landscape ----------------------------------------------------------------
const q = await phone(844, 390, base);
await open(q);
await q.page.waitForTimeout(2500);
await q.page.locator('[data-testid="drive-full-arm"]').tap();
await q.page.locator('[data-testid="drive-full-arm-yes"]').tap(); // new browser session: asked again
await q.page.waitForTimeout(300);
await touch(q.cdp, "touchStart", [[150, 260, 1], [700, 260, 2]]);
for (let i = 1; i <= 6; i++) {
  await touch(q.cdp, "touchMove", [[150 - i * 4, 260 - i * 8, 1], [700 + i * 9, 260, 2]]);
  await q.page.waitForTimeout(40);
}
await q.page.waitForTimeout(300);
console.log("landscape      ", JSON.stringify((await state(q))?.commanded));
await shot(q.page, "6_landscape_driving");
await touch(q.cdp, "touchEnd", []);
await q.page.context().close();

// --- all cameras (stills, six Spot cameras) -------------------------------------
const g = await phone(390, 844, gridBase);
await open(g);
await g.page.locator('[data-testid="drive-full-view"]').tap();
await g.page.locator(".dfs-menu .segmented button", {hasText: "Stills"}).tap();
await g.page.getByRole("menuitemradio", {name: "All cameras"}).tap();
await g.page.waitForTimeout(2500);
await shot(g.page, "7_all_cameras");
await g.page.context().close();

console.log("page errors:", errors.length ? errors : "none");
await browser.close();
