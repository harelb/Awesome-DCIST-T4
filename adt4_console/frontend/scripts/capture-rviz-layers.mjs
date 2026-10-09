// Screenshots of the RViz-replacement layers against a real backend + live robot:
// occupancy grid, executor paths, camera tab, DSG stats.
//
//   CONSOLE_URL=http://127.0.0.1:8095/ SHOT_DIR=/tmp/shots ROBOT=euclid PRIOR=neo4j:prior \
//     node scripts/capture-rviz-layers.mjs
import {mkdirSync} from "node:fs";
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8095/";
const shotDir = process.env.SHOT_DIR ?? "shots/rviz-layers";
const robot = process.env.ROBOT ?? "euclid";
const prior = process.env.PRIOR ?? "neo4j:prior";
const chrome =
  process.env.CHROMIUM_PATH ??
  join(process.env.HOME, ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome");
mkdirSync(shotDir, {recursive: true});

const browser = await chromium.launch({
  executablePath: chrome,
  headless: true,
  args: ["--no-sandbox", "--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=vulkan", "--enable-features=Vulkan"],
});
const page = await browser.newPage({viewport: {width: 1600, height: 1000}});
const errors = [];
page.on("pageerror", e => errors.push(String(e)));
page.on("console", m => m.type() === "error" && errors.push(m.text()));
const shot = async name => {
  await page.screenshot({path: join(shotDir, `${name}.png`)});
  console.log("shot", name);
};
const setSource = async (id, on) => {
  const row = page.locator(".src-list li", {hasText: id}).first();
  if (!(await row.count())) return;
  const box = row.locator(".toggle input[type=checkbox]").first();
  if ((await box.isChecked()) !== on) await row.locator(".toggle").first().click();
};

await page.goto(`${base}#/viewer`);
await page.locator("canvas").first().waitFor({timeout: 60_000});
await page.waitForTimeout(2000);
// live robot only: its mesh + graph + occupancy grid
await setSource(prior, false);
await setSource(`live:${robot}`, true);
await page.waitForTimeout(6000);
await page.getByRole("tab", {name: "Top", exact: true}).click().catch(() => {});
await page.getByLabel("Follow robot").selectOption(robot).catch(() => {});
await page.waitForTimeout(3000);
await shot("p6-live-occupancy-top");

await page.getByRole("tab", {name: "Camera", exact: true}).click();
await page.waitForTimeout(4000);
await shot("p6-camera-rgb");
const camSel = page.locator("select").filter({hasText: "overlay"}).first();
if (await camSel.count()) {
  await camSel.selectOption("overlay");
  await page.waitForTimeout(3000);
  await shot("p6-camera-overlay");
}

// stats: Inspect tab with nothing selected, prior + live enabled
await setSource(prior, true);
await page.getByRole("tab", {name: "3D", exact: true}).click().catch(() => {});
await page.getByRole("tab", {name: "Inspect", exact: true}).click();
await page.waitForTimeout(5000);
await shot("p6-stats");

console.log("errors:", errors.length ? errors : "none");
await browser.close();
