// Capture a mission executing in the viewer (real backend + executor).
//
//   CONSOLE_URL=http://127.0.0.1:8095/ SHOT_DIR=/tmp/shots ROBOT=euclid PRIOR=neo4j:prior \
//     GOAL='(and (visited-object o8) (visited-object o246))' node scripts/capture-mission.mjs
//
// Sends the PDDL goal from the Plan tab, then screenshots while the executor
// reports results; also records the robot-state trail for the report.
import {mkdirSync, writeFileSync} from "node:fs";
import {join} from "node:path";
import {chromium} from "playwright-core";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8095/";
const shotDir = process.env.SHOT_DIR ?? "shots/mission";
const robot = process.env.ROBOT ?? "euclid";
const prior = process.env.PRIOR ?? "neo4j:prior";
const goal = process.env.GOAL ?? "(and (visited-object o8) (visited-object o246) (visited-object o328))";
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
const get = async path => page.evaluate(async p => (await fetch(p)).json(), `${base}api/${path}`);

await page.goto(`${base}#/viewer`);
await page.locator("canvas").first().waitFor({timeout: 60_000});
await page.waitForTimeout(2000);
for (const id of [prior, `live:${robot}`]) {
  const row = page.locator(".src-list li", {hasText: id}).first();
  if (!(await row.count())) continue;
  const box = row.locator(".toggle input[type=checkbox]").first();
  if (!(await box.isChecked())) await row.locator(".toggle").first().click();
}
await page
  .waitForFunction(() => {
    const m = [...(document.querySelector(".vstatus")?.textContent ?? "").matchAll(/tiles (\d+)\/(\d+)/g)];
    return m.length > 0 && m.every(x => x[1] === x[2]);
  }, undefined, {timeout: 90_000})
  .catch(() => console.log("  (tiles still streaming)"));
// the live source here re-publishes the prior; hide its graph so the overlay stays readable
await page.getByRole("tab", {name: "Top", exact: true}).click().catch(() => {});
await page.getByLabel("Follow robot").selectOption(robot).catch(e => console.log("  follow:", String(e).slice(0, 80)));

// send the mission through the UI: Plan tab -> PDDL -> type goal -> send
await page.getByRole("tab", {name: "Plan", exact: true}).click();
await page.getByRole("tab", {name: "PDDL", exact: true}).click();
const before = (await get(`robots/${robot}/plan`)).plan?.plan_id;
const box = page.locator(".plan-panel textarea, textarea").first();
await box.fill(goal);
await page.getByLabel("PDDL plugin").selectOption("visit_objects_pddl").catch(() => {});
await page.getByRole("button", {name: /Send PDDL goal/}).click();
let planId = before;
for (let i = 0; i < 40 && planId === before; i++) {
  await page.waitForTimeout(500);
  planId = (await get(`robots/${robot}/plan`)).plan?.plan_id;
}
console.log("plan", planId);

const trail = [];
let shotN = 0;
const t0 = Date.now();
while (Date.now() - t0 < Number(process.env.MAX_S ?? 150) * 1000) {
  const st = await get(`live/${robot}/state`).catch(() => null);
  const plan = await get(`robots/${robot}/plan`);
  const res = plan.results.filter(r => r.plan_id === planId);
  if (st?.pose) trail.push({t: (Date.now() - t0) / 1000, x: st.pose.x, y: st.pose.y, done: res.length});
  if (trail.length % 8 === 1) {
    await page.screenshot({path: join(shotDir, `mission-${String(++shotN).padStart(2, "0")}.png`)});
    console.log(`shot ${shotN}: t=${trail.at(-1)?.t.toFixed(0)}s done=${res.length}/${plan.plan?.actions.length}`);
  }
  if (plan.plan && res.length >= plan.plan.actions.length) {
    await page.waitForTimeout(1500);
    await page.getByRole("tab", {name: "3D", exact: true}).click().catch(() => {});
    await page.waitForTimeout(1500);
    await page.screenshot({path: join(shotDir, "mission-done.png")});
    console.log("mission complete:", res.map(r => `${r.action_index}:${r.status}`).join(" "));
    break;
  }
  await page.waitForTimeout(1500);
}
writeFileSync(join(shotDir, "trail.json"), JSON.stringify(trail));
console.log("errors:", errors.length ? errors : "none");
await browser.close();
