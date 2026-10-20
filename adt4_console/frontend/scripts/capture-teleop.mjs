// Screenshot tour of the Teleop tab against a real console + executor (sim /
// FakeSpot):
//
//   CONSOLE_URL=http://127.0.0.1:8197/ SHOT_DIR=/tmp/shots ROBOT=hamilton node scripts/capture-teleop.mjs
//
// Docked tab, floating window, arm confirmation, ARMED + driving with W held,
// disarm on blur, Stand/Sit results, phone layout. Prints the teleop state the
// console reported at each step.
import {mkdirSync} from "node:fs";
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8197/";
const shotDir = process.env.SHOT_DIR ?? "shots/teleop";
const robot = process.env.ROBOT ?? "hamilton";
const chrome = process.env.CHROMIUM_PATH ?? join(process.env.HOME, ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome");
mkdirSync(shotDir, {recursive: true});

const browser = await chromium.launch({
  executablePath: chrome,
  headless: true,
  args: ["--no-sandbox", "--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=vulkan", "--enable-features=Vulkan"],
});
const errors = [];
const state = async page => (await page.evaluate(async u => (await fetch(u)).json(), `${base}api/robots/${robot}/teleop`)).state;
const log = async (page, label) => {
  const s = await state(page);
  console.log(label.padEnd(22), s ? JSON.stringify({active: s.active, standing: s.standing, commanded: s.commanded, event: s.last_event, reject: s.last_reject}) : "no state");
};
const prefix = process.env.SHOT_PREFIX ?? "teleop_";
const shot = async (page, name) => page.screenshot({path: join(shotDir, `${prefix}${name}.png`)});

const page = await browser.newPage({viewport: {width: 1600, height: 1000}});
page.on("pageerror", e => errors.push(String(e)));
await page.goto(`${base}#/viewer`);
await page.locator("canvas").first().waitFor({timeout: 60_000});
await page.getByRole("tab", {name: "Teleop"}).click().catch(async () => page.locator(".vright-head button", {hasText: "Teleop"}).first().click());
await page.locator(".teleop-panel").waitFor();
await page.waitForTimeout(1500);
await shot(page, "1_docked");
await log(page, "docked");

// pop out into a floating window
await page.locator(".pop-btn").click();
await page.locator(".float-win .teleop-panel").waitFor();
await page.waitForTimeout(500);
await shot(page, "2_floating");

// arm: confirmation first time in the session
await page.locator('[data-testid="teleop-drive"]').click();
await page.locator('[data-testid="teleop-arm-yes"]').waitFor();
await shot(page, "3_confirm");
await page.locator('[data-testid="teleop-arm-yes"]').click();
await page.locator(".teleop-armed").waitFor();
await page.keyboard.down("KeyW");
await page.keyboard.down("KeyQ");
await page.waitForTimeout(1500);
await shot(page, "4_armed_driving");
await log(page, "driving W+Q");
await page.keyboard.up("KeyQ");
await page.keyboard.up("KeyW");
await page.waitForTimeout(1200);
await log(page, "keys released");

// click on the map: the panel loses focus -> disarmed
await page.mouse.click(500, 500);
await page.waitForTimeout(800);
await shot(page, "5_disarmed_on_blur");
await log(page, "after blur");

// posture buttons
const btn = name => page.locator(".float-win .teleop-buttons button", {hasText: new RegExp(`^\\s*${name}\\s*$`)});
await btn("Sit").click();
await page.waitForTimeout(1500);
await log(page, "after Sit");
await btn("Stand").click();
await page.waitForTimeout(1500);
await shot(page, "6_stand_result");
await log(page, "after Stand");

// phone layout (docked bottom sheet)
const phone = await browser.newPage({viewport: {width: 390, height: 844}, isMobile: true, hasTouch: true});
await phone.goto(`${base}#/viewer`);
await phone.waitForTimeout(2500);
await phone.getByLabel("Toggle side panel").first().click();
await phone.waitForTimeout(500);
await phone.locator(".vright-head button", {hasText: "Teleop"}).first().click({timeout: 10_000}).catch(e => errors.push(`phone tab: ${e}`));
await phone.waitForTimeout(1200);
await phone.screenshot({path: join(shotDir, `${prefix}7_phone.png`)});

console.log("errors:", errors.length ? errors : "none");
await browser.close();
