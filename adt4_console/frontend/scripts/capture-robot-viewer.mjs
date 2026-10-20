// Screenshot tour of the Robot viewer's focus + drive layout, against the mock
// backend (two live robots with cameras and teleop) or a real console:
//
//   npm run mock &   # or MOCK_PORT=8093 node scripts/mock-server.mjs
//   CONSOLE_URL=http://127.0.0.1:8090/ SHOT_DIR=/tmp/shots ROBOT=euclid node scripts/capture-robot-viewer.mjs
//
// 1 all robots (standard layout, camera tab: one camera per robot),
// 2 one robot focused (drive layout: map + camera + teleop), 3 armed + driving,
// 4 clicking the map disarms, 5 phone width (camera picture-in-picture, teleop
// sheet), 6 phone all robots, 7 narrow desktop. Never arms unless ARM=1 (do not arm against a real robot unattended).
import {mkdirSync} from "node:fs";
import {join} from "node:path";
import {homedir} from "node:os";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8090/";
const shotDir = process.env.SHOT_DIR ?? "shots/robot-viewer";
const robot = process.env.ROBOT ?? "euclid";
const arm = process.env.ARM === "1";
const chrome = process.env.CHROMIUM_PATH ?? join(homedir(), ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome");
mkdirSync(shotDir, {recursive: true});
const url = route => {
  const u = new URL(base);
  u.hash = `#${route}`;
  return u.toString();
};

const browser = await chromium.launch({
  executablePath: chrome,
  headless: true,
  args: ["--no-sandbox", "--disable-dev-shm-usage", "--enable-webgl", "--ignore-gpu-blocklist", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});
const errors = [];
const shots = [];
const shot = async (page, name) => {
  const path = join(shotDir, `${name}.png`);
  await page.screenshot({path});
  shots.push(path);
};
const watch = page => {
  page.on("pageerror", e => errors.push(String(e)));
  page.on("console", m => m.type() === "error" && !/Failed to load resource|status of (4|5)\d\d/.test(m.text()) && errors.push(m.text()));
};
const focus = async (page, name) => {
  await page.getByTestId("focus-picker").click();
  await page.getByRole("listbox", {name: "Robot focus"}).getByRole("option", {name: new RegExp(`^${name === "all" ? "All robots" : name}`)}).first().click();
  await page.waitForTimeout(800);
};

try {
  const ctx = await browser.newContext({viewport: {width: 1600, height: 1000}});
  const page = await ctx.newPage();
  watch(page);
  await page.goto(url("/viewer"));
  await page.locator("canvas").first().waitFor({timeout: 60_000});
  await page.evaluate(() => localStorage.clear());
  await page.reload();
  await page.locator("canvas").first().waitFor({timeout: 60_000});
  await page.waitForTimeout(2500);
  const live = page.locator(".src-list li", {hasText: `live:${robot}`}).first();
  if (await live.count()) await live.locator(".toggle").click().catch(() => undefined);
  await focus(page, "all");
  await page.getByRole("tab", {name: "Camera", exact: true}).click();
  await page.waitForTimeout(2500);
  await shot(page, "1_all_robots");

  await focus(page, robot);
  await page.waitForTimeout(2500);
  await shot(page, "2_focused_drive");

  if (arm) {
    await page.locator('[data-testid="teleop-drive"]').click();
    await page.locator('[data-testid="teleop-arm-yes"]').click().catch(() => undefined);
    await page.locator(".teleop-armed").waitFor();
    await page.keyboard.down("KeyW");
    await page.waitForTimeout(800);
    await shot(page, "3_armed_driving");
    await page.keyboard.up("KeyW");
    const box = await page.locator(".vcanvas canvas").first().boundingBox();
    if (box) await page.mouse.click(box.x + box.width * 0.5, box.y + box.height * 0.6);
    await page.waitForTimeout(500);
    if (await page.locator(".teleop-armed").count()) errors.push("still armed after clicking the map");
    await shot(page, "4_map_click_disarmed");
  }

  // narrow desktop / tablet (641-900 px): docked panels overlay the canvas
  await page.setViewportSize({width: 860, height: 1000});
  await page.waitForTimeout(1500);
  await shot(page, "7_narrow_drive");

  const phone = await browser.newContext({viewport: {width: 390, height: 844}, deviceScaleFactor: 2, isMobile: true, hasTouch: true});
  const pp = await phone.newPage();
  watch(pp);
  await pp.goto(url("/viewer"));
  await pp.locator("canvas").first().waitFor({timeout: 60_000});
  await pp.waitForTimeout(2500);
  await focus(pp, robot);
  await pp.waitForTimeout(2500);
  await shot(pp, "5_phone_drive");
  await focus(pp, "all");
  await pp.waitForTimeout(1500);
  await shot(pp, "6_phone_all");
} finally {
  await browser.close();
}
console.log(JSON.stringify({shots, errors}, null, 1));
if (errors.length) process.exit(1);
