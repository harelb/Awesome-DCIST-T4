// Screenshot the Camera tab's live Video mode against a real backend + robot.
//
//   CONSOLE_URL=http://127.0.0.1:8095/ SHOT_DIR=/tmp/shots ROBOT=euclid CAM=rgb \
//     FPS=10 CODEC=h264 node scripts/capture-video.mjs
import {mkdirSync} from "node:fs";
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8095/";
const shotDir = process.env.SHOT_DIR ?? "shots/video";
const robot = process.env.ROBOT ?? "euclid";
const cam = process.env.CAM ?? "rgb";
const fps = process.env.FPS ?? "10";
const codec = process.env.CODEC ?? "auto";
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

await page.goto(`${base}#/viewer`);
await page.locator("canvas").first().waitFor({timeout: 60_000});
const row = page.locator(".src-list li", {hasText: `live:${robot}`}).first();
if ((await row.count()) && !(await row.locator(".toggle input[type=checkbox]").first().isChecked()))
  await row.locator(".toggle").first().click();
await page.getByLabel("Follow robot").selectOption(robot).catch(() => {});
await page.getByRole("tab", {name: "Camera", exact: true}).click();
await page.getByLabel("Camera stream").selectOption(cam).catch(() => {});
await page.getByRole("group", {name: "Camera mode"}).getByRole("button", {name: "Video"}).click();
await page.getByLabel("Video frame rate").selectOption(fps).catch(() => {});
if (codec !== "auto") await page.getByLabel("Codec").selectOption(codec).catch(() => {});
const supported = await page.evaluate(async () => {
  const out = {};
  for (const [name, c] of [["h264", "avc1.42E01E"], ["vp8", "vp8"]]) {
    try {
      out[name] = (await VideoDecoder.isConfigSupported({codec: c})).supported;
    } catch {
      out[name] = false;
    }
  }
  return out;
});
console.log("browser decoder support:", JSON.stringify(supported));
await page.waitForTimeout(12000);
await page.screenshot({path: join(shotDir, `video-${cam}-${fps}fps.png`)});
const stats = await page.locator(".video-stats, .camera-meta").first().textContent().catch(() => "");
console.log("stats:", (stats ?? "").replace(/\s+/g, " ").trim());
console.log("errors:", errors.length ? errors : "none");
await browser.close();
