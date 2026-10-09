// Screenshot tour against a REAL console backend (safe: dry-run only, no launches).
//
//   CONSOLE_URL=http://127.0.0.1:8095/ SHOT_DIR=/tmp/shots LIVE=euclid PRIOR=neo4j:prior \
//     node scripts/capture-real.mjs
//
// Viewer shots overlay the prior source and the live robot.  The typed-confirm
// shot starts an ingest job and then answers "no" (the DB is left untouched).
import {mkdirSync} from "node:fs";
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8095/";
const shotDir = process.env.SHOT_DIR ?? "shots/real";
const live = process.env.LIVE ?? "euclid";
const prior = process.env.PRIOR ?? "neo4j:prior";
const chrome =
  process.env.CHROMIUM_PATH ??
  join(process.env.HOME, ".cache/ms-playwright/chromium-1234/chrome-linux64/chrome");
mkdirSync(shotDir, {recursive: true});

const browser = await chromium.launch({
  executablePath: chrome,
  headless: true,
  args: ["--no-sandbox", "--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=vulkan",
    "--enable-features=Vulkan", "--enable-unsafe-webgpu"],
});
const page = await browser.newPage({viewport: {width: 1600, height: 1000}});
const errors = [];
page.on("pageerror", e => errors.push(String(e)));
page.on("console", m => m.type() === "error" && errors.push(m.text()));
const url = p => `${base}#${p}`;
const shot = async name => {
  await page.screenshot({path: join(shotDir, `${name}.png`)});
  console.log("shot", name);
};
const api = async (path, body) =>
  page.evaluate(
    async ([p, b]) => {
      const r = await fetch(p, b ? {method: "POST", headers: {"content-type": "application/json"}, body: JSON.stringify(b)} : {});
      return r.json();
    },
    [`${base}api/${path}`, body],
  );
const tilesLoaded = () =>
  page
    .waitForFunction(() => {
      const t = document.querySelector(".vstatus")?.textContent ?? "";
      const m = [...t.matchAll(/tiles (\d+)\/(\d+)/g)];
      return m.length > 0 && m.every(x => x[1] === x[2]);
    }, undefined, {timeout: 90_000})
    .catch(() => console.log("  (tiles still streaming)"));

// ---------------------------------------------------------------- pages
await page.goto(url("/fleet"));
await page.waitForTimeout(4000);
await shot("01-fleet");

await page.goto(url("/launch"));
await page.waitForTimeout(1500);
await page.getByRole("button", {name: /Mapping/}).first().click().catch(() => {});
await page.waitForTimeout(800);
const exp = page.getByLabel(/Experiment/i).first();
if (await exp.count()) await exp.fill("10092026_pennovation_web_demo");
await page.getByRole("button", {name: /Dry run/i}).first().click().catch(() => {});
await page.waitForTimeout(1500);
await shot("02-launch-dryrun");

await page.goto(url("/maps"));
await page.waitForTimeout(6000);
await shot("03-maps");

await page.goto(url("/network"));
await page.waitForTimeout(4000);
await shot("04-network");

await page.goto(url("/config"));
await page.waitForTimeout(2000);
await shot("05-config");

// ---------------------------------------------------------------- viewer
await page.goto(url("/viewer"));
await page.locator("canvas").first().waitFor({timeout: 60_000});
const renderer = await page.evaluate(() => {
  const gl = document.createElement("canvas").getContext("webgl2");
  const ext = gl?.getExtension("WEBGL_debug_renderer_info");
  return ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : "unknown";
});
console.log("WebGL renderer:", renderer);
await page.waitForTimeout(2000);
// enable prior + live sources: rows are <li> with a .toggle label wrapping a checkbox
for (const id of [prior, `live:${live}`]) {
  const row = page.locator(".src-list li", {hasText: id}).first();
  if (!(await row.count())) continue;
  const box = row.locator(".toggle input[type=checkbox]").first();
  if (!(await box.isChecked())) await row.locator(".toggle").first().click();
}
await tilesLoaded();
await page.waitForTimeout(2500);
await shot("06-viewer-prior-plus-live");

await page.getByRole("tab", {name: "Top", exact: true}).click().catch(() => {});
await page.waitForTimeout(2500);
await shot("07-viewer-topdown");
await page.getByRole("tab", {name: "3D", exact: true}).click().catch(() => {});
await page.waitForTimeout(1200);

await page.getByRole("tab", {name: "Live link", exact: true}).click().catch(() => {});
await page.waitForTimeout(6000);
await shot("08-viewer-live-link");

const search = page.getByLabel("Search graph");
await search.fill("sign");
await page.waitForTimeout(400);
await search.press("Enter");
await page.waitForTimeout(2500);
await shot("09-viewer-search-inspect");

// a real plan: goto two objects via the API, then show the Plan tab
await api(`robots/${live}/goal/pddl`, {plugin: "visit_objects_pddl", goal: "(and (visited-object o2) (visited-object o10))"});
await page.waitForTimeout(6000);
await page.getByRole("tab", {name: "Plan", exact: true}).click().catch(() => {});
await page.waitForTimeout(2500);
await shot("10-viewer-plan");

await page.getByRole("tab", {name: "Chat", exact: true}).click().catch(() => {});
await page.getByLabel("Chat message").fill("go to the sign").catch(() => {});
await page.getByLabel("Chat message").press("Enter").catch(() => {});
await page.locator(".msg-goal").waitFor({timeout: 20_000}).catch(() => {});
await page.waitForTimeout(800);
await shot("11-viewer-chat");

// typed confirm: start an ingest, screenshot the modal, answer "no"
const job = await api("maps/ingest", {path: process.env.INGEST_PATH, target: prior.split(":")[1]});
await page.locator(".req-text, [data-testid=confirm-yes]").first().waitFor({timeout: 30_000}).catch(() => {});
const req = page.locator(".req-text input, input[aria-label*=onfirm]").first();
if (await req.count()) await req.fill("wip");
await page.waitForTimeout(600);
await shot("12-typed-confirm-wipe");
await api(`jobs/${job.job_id}/confirm`, {answer: false});
await page.waitForTimeout(1500);
await page.getByRole("button", {name: "Jobs"}).click().catch(() => {});
await page.waitForTimeout(1500);
await shot("13-jobs-drawer");

console.log("errors:", errors.length ? errors : "none");
await browser.close();
