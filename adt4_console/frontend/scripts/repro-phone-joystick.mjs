// Phone Teleop-tab joystick check (touch-emulated phone, sheet layout):
// joystick with real touch events (CDP), report what the robot got.
//   CONSOLE_URL=http://127.0.0.1:8291/ node scripts/repro-phone-joystick.mjs
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8291/";
const robot = process.env.ROBOT ?? "euclid";
const shotDir = process.env.SHOT_DIR ?? "/tmp";
const chrome = process.env.CHROMIUM_PATH ?? join(process.env.HOME, ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome");
const browser = await chromium.launch({executablePath: chrome, headless: true, args: ["--no-sandbox", "--use-angle=vulkan", "--enable-features=Vulkan", "--ignore-gpu-blocklist"]});
const ctx = await browser.newContext({viewport: {width: 390, height: 844}, deviceScaleFactor: 2, isMobile: true, hasTouch: true});
await ctx.addInitScript(r => {
  localStorage.setItem("adt4.viewer.focus.v1", r);
  sessionStorage.setItem("adt4.teleop.armConfirmed", "1");
}, robot);
const page = await ctx.newPage();
page.on("pageerror", e => console.log("pageerror", String(e)));
const cdp = await ctx.newCDPSession(page);
const state = async () => (await page.evaluate(async u => (await fetch(u)).json(), `${base}api/robots/${robot}/teleop`)).state;
const touch = (type, pts) => cdp.send("Input.dispatchTouchEvent", {type, touchPoints: pts.map(([x, y], id) => ({x, y, id}))});

await page.goto(`${base}#/viewer`);
await page.locator("canvas").first().waitFor({timeout: 60_000});
await page.waitForTimeout(1500);
console.log("focus key", await page.evaluate(() => Object.keys(localStorage)));
const tabs = page.locator(".vsheet .vright-head button", {hasText: "Teleop"});
if (!(await tabs.count())) await page.getByRole("button", {name: "Toggle side panel"}).tap();
await page.locator(".vsheet .vright-head button", {hasText: "Teleop"}).first().tap();
await page.locator(".teleop-panel").waitFor();
await page.locator('[data-testid="teleop-drive"]').tap();
await page.waitForTimeout(300);
console.log("armed after tap:", await page.locator(".teleop-armed").count());
const joy = page.locator(".joystick");
await joy.scrollIntoViewIfNeeded();
const b = await joy.boundingBox();
console.log("joystick box", b);
const cx = b.x + b.width / 2;
const cy = b.y + b.height / 2;
await touch("touchStart", [[cx, cy]]);
for (let i = 1; i <= 6; i++) {
  await touch("touchMove", [[cx, cy - (i * b.height) / 12]]);
  await page.waitForTimeout(60);
}
await page.waitForTimeout(500);
console.log("armed while dragging:", await page.locator(".teleop-armed").count(), "note:", await page.locator(".teleop-panel .small.muted").allTextContents());
console.log("state while dragging", JSON.stringify((await state())?.commanded));
await page.screenshot({path: join(shotDir, "repro_dragging.png")});
await touch("touchEnd", []);
await page.waitForTimeout(400);
console.log("state after release", JSON.stringify((await state())?.commanded));
await browser.close();
