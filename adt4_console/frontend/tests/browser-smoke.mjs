#!/usr/bin/env node
// Browser smoke test for the ADT4 console.
//
//   CONSOLE_URL=http://127.0.0.1:8090/?token=XYZ SHOT_DIR=/tmp/shots node tests/browser-smoke.mjs
//
// Loads every page, waits for data + (on the viewer) the WebGL canvas, asserts no
// JavaScript errors, and saves a screenshot per page into SHOT_DIR. Read-only by
// default; SMOKE_INTERACT=1 additionally drives a launch + typed-confirm flow
// (only use that against scripts/mock-server.mjs: it really launches).
import {chromium} from "playwright-core";
import {existsSync, mkdirSync} from "node:fs";
import {homedir} from "node:os";
import {join} from "node:path";

const base = process.env.CONSOLE_URL ?? "http://127.0.0.1:8090/";
const shotDir = process.env.SHOT_DIR ?? "";
const interact = process.env.SMOKE_INTERACT === "1";
const width = Number(process.env.SMOKE_WIDTH ?? 1600);
const height = Number(process.env.SMOKE_HEIGHT ?? 1000);
if (shotDir) mkdirSync(shotDir, {recursive: true});

function findChrome() {
  if (process.env.CHROMIUM_PATH) return process.env.CHROMIUM_PATH;
  const pw = join(homedir(), ".cache", "ms-playwright");
  const candidates = [
    join(pw, "chromium-1234", "chrome-linux64", "chrome"),
    "/usr/bin/google-chrome",
    "/usr/bin/chromium",
    "/snap/bin/chromium",
    "/usr/bin/chromium-browser",
  ];
  for (const c of candidates) if (existsSync(c)) return c;
  return chromium.executablePath();
}

// URL of a hash route, keeping ?token=... from CONSOLE_URL
function pageUrl(route) {
  const u = new URL(base);
  u.hash = `#${route}`;
  return u.toString();
}

const browser = await chromium.launch({
  executablePath: findChrome(),
  headless: true,
  args: ["--no-sandbox", "--disable-dev-shm-usage", "--enable-webgl", "--ignore-gpu-blocklist", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
});

const jsErrors = [];
const httpErrors = [];
const warnings = [];
const results = [];

async function settle(page, ms = 600) {
  await page.waitForLoadState("networkidle", {timeout: 15_000}).catch(() => undefined);
  await page.waitForTimeout(ms);
}

async function shot(page, name) {
  if (!shotDir) return;
  const path = join(shotDir, `${name}.png`);
  await page.screenshot({path, fullPage: false});
  results.push({name, path});
}

try {
  const ctx = await browser.newContext({viewport: {width, height}, deviceScaleFactor: 1});
  const page = await ctx.newPage();
  page.on("pageerror", e => jsErrors.push(`pageerror: ${e.message}`));
  page.on("console", msg => {
    if (msg.type() === "warning") warnings.push(msg.text());
    if (msg.type() !== "error") return;
    const text = msg.text();
    // the browser itself logs failed HTTP requests (e.g. 503 when ROS is down):
    // those are expected states the UI renders, not script errors
    if (/Failed to load resource|status of (4|5)\d\d/.test(text)) httpErrors.push(text);
    else jsErrors.push(`console: ${text}`);
  });

  // first load establishes the token in sessionStorage
  await page.goto(pageUrl("/fleet"), {waitUntil: "domcontentloaded", timeout: 60_000});
  await page.locator(".sidenav").waitFor({timeout: 30_000});

  const pages = [
    {route: "/fleet", name: "fleet", ready: ".page-head h1"},
    {route: "/launch", name: "launch", ready: ".launch-bar"},
    {route: "/maps", name: "maps", ready: ".page-head h1"},
    {route: "/network", name: "network", ready: ".page-head h1"},
    {route: "/config", name: "config", ready: ".page-head h1"},
  ];
  for (const p of pages) {
    await page.goto(pageUrl(p.route), {waitUntil: "domcontentloaded"});
    await page.locator(p.ready).first().waitFor({timeout: 30_000});
    // wait until no spinners remain (or give up after 15 s: slow ssh scans)
    await page.waitForFunction(() => !document.querySelector(".content .state-box .spinner"), undefined, {timeout: 15_000}).catch(() => undefined);
    await settle(page);
    await shot(page, p.name);
  }

  // viewer: canvas + some content
  await page.goto(pageUrl("/viewer"), {waitUntil: "domcontentloaded"});
  await page.locator("canvas").first().waitFor({timeout: 60_000});
  await page
    .waitForFunction(() => /[1-9][\d,]* nodes/.test(document.querySelector(".vstatus")?.textContent ?? ""), undefined, {timeout: 60_000})
    .catch(() => undefined);
  // let tiles stream in (stats text "tiles a/b" with a == b), max 60 s
  await page
    .waitForFunction(
      () => {
        const t = document.querySelector(".vstatus")?.textContent ?? "";
        const m = [...t.matchAll(/(\d+)\/(\d+)/g)];
        return m.length === 0 ? true : m.every(x => x[1] === x[2]);
      },
      undefined,
      {timeout: 60_000},
    )
    .catch(() => undefined);
  await settle(page, 1500);
  const blank = await page.evaluate(() => {
    const c = document.querySelector("canvas");
    return !c || c.width === 0 || c.height === 0;
  });
  if (blank) jsErrors.push("viewer canvas has zero size");
  await shot(page, "viewer");

  // search: highlight + fly to the first hit
  const search = page.getByLabel("Search graph");
  if (await search.count()) {
    await search.fill(process.env.SMOKE_QUERY ?? "sign");
    await page.waitForTimeout(400);
    await search.press("Enter");
    await page.waitForTimeout(1800);
    await shot(page, "viewer-search");
  }
  // side panel tabs (read-only)
  for (const tab of ["Plan", "Live link", "Chat"]) {
    const t = page.getByRole("tab", {name: tab, exact: true});
    if (await t.count()) {
      await t.click();
      await settle(page, 700);
      await shot(page, `viewer-${tab.toLowerCase().replace(/\s+/g, "-")}`);
    }
  }
  // robot focus: the first robot (read-only: focusing never arms teleop), the
  // drive layout it brings up, then back to all robots
  const picker = page.getByTestId("focus-picker");
  if (await picker.count()) {
    await picker.click();
    const opts = page.getByRole("listbox", {name: "Robot focus"}).getByRole("option");
    if ((await opts.count()) > 1) {
      await opts.nth(1).click();
      await settle(page, 1500);
      if (!(await page.locator(".vright.drive .drive-cam").count())) jsErrors.push("focusing a robot did not bring up the drive layout");
      await shot(page, "viewer-focus-drive");
      await picker.click();
      await page.getByRole("listbox", {name: "Robot focus"}).getByRole("option").first().click();
      await settle(page, 500);
    } else await page.keyboard.press("Escape");
  }
  // top-down
  const top = page.getByRole("tab", {name: "Top", exact: true});
  if (await top.count()) {
    await top.click();
    await page.waitForTimeout(1500);
    await shot(page, "viewer-topdown");
  }

  // jobs drawer
  await page.getByRole("button", {name: "Jobs"}).click();
  await page.locator(".drawer").waitFor({timeout: 10_000});
  await page.locator(".joblist li").first().click().catch(() => undefined);
  await settle(page, 500);
  await shot(page, "jobs-drawer");
  await page.getByRole("button", {name: "Close jobs"}).click();

  if (interact) {
    // viewer: overlay the live robot, goto picking, chat proposal
    await page.goto(pageUrl("/viewer"), {waitUntil: "domcontentloaded"});
    await page.locator("canvas").first().waitFor({timeout: 60_000});
    await settle(page, 1500);
    const live = page.locator(".src-list li", {hasText: "live:"}).first();
    if (await live.count()) {
      await live.locator(".toggle").click();
      await settle(page, 2500);
    }
    await page.getByRole("tab", {name: "Plan", exact: true}).click();
    await page.getByRole("button", {name: "Pick on map"}).click();
    const box = await page.locator(".vcanvas canvas").first().boundingBox();
    if (box) {
      for (const [fx, fy] of [[0.45, 0.55], [0.6, 0.45], [0.35, 0.35]]) {
        await page.mouse.click(box.x + box.width * fx, box.y + box.height * fy);
        await page.waitForTimeout(300);
      }
    }
    await page.waitForTimeout(800);
    await shot(page, "viewer-goto");
    await page.getByRole("button", {name: "Done"}).click().catch(() => undefined);
    await page.getByRole("tab", {name: "Chat", exact: true}).click();
    await page.getByLabel("Chat message").fill("find the fire extinguisher");
    await page.getByLabel("Chat message").press("Enter");
    await page.locator(".msg-goal").waitFor({timeout: 15_000}).catch(() => undefined);
    await page.waitForTimeout(500);
    await shot(page, "viewer-chat-proposal");

    // maps: verify matrix + typed "wipe" confirm for a Neo4j ingest
    await page.goto(pageUrl("/maps"), {waitUntil: "domcontentloaded"});
    await page.getByRole("button", {name: "Verify"}).first().waitFor({timeout: 20_000});
    await page.getByRole("button", {name: "Verify"}).first().click();
    await page.getByRole("button", {name: "Compare checksums"}).click();
    await page.getByRole("button", {name: "Close jobs"}).click().catch(() => undefined);
    await page.locator(".checksum").first().waitFor({timeout: 20_000});
    await page.waitForTimeout(400);
    await shot(page, "maps-verify");
    await page.getByRole("button", {name: "Load into Neo4j"}).first().click();
    await page.getByRole("button", {name: "Continue"}).click();
    await page.locator(".req-text").waitFor({timeout: 15_000});
    await page.getByLabel("Confirmation text").fill("wip");
    await shot(page, "ingest-confirm");
    if (!(await page.getByTestId("confirm-yes").isDisabled())) jsErrors.push("ingest confirm enabled with wrong text");
    await page.getByLabel("Confirmation text").fill("wipe");
    await page.getByTestId("confirm-yes").click();
    await page.waitForTimeout(2000);
    await shot(page, "ingest-job");
    await page.getByRole("button", {name: "Close jobs"}).click().catch(() => undefined);

    await page.goto(pageUrl("/launch"), {waitUntil: "domcontentloaded"});
    await page.locator(".launch-bar").waitFor();
    await settle(page);
    await page.getByRole("button", {name: "Dry run"}).click();
    await page.locator(".dryrun").waitFor({timeout: 10_000});
    await shot(page, "launch-dryrun");
    await page.getByRole("button", {name: "Launch", exact: true}).click();
    await page.locator(".modal").waitFor({timeout: 15_000});
    const yes = page.getByTestId("confirm-yes");
    if (!(await yes.isDisabled())) jsErrors.push("typed-confirm Yes was enabled before typing");
    await shot(page, "confirm-modal");
    const req = await page.locator(".req-text").textContent();
    await page.getByLabel("Confirmation text").fill(req ?? "");
    if (await yes.isDisabled()) jsErrors.push("typed-confirm Yes still disabled after typing the required text");
    await yes.click();
    await page.waitForTimeout(2500);
    await shot(page, "launch-job");
  }

  if (jsErrors.length) throw new Error(`browser errors:\n  ${jsErrors.join("\n  ")}`);
  console.log(JSON.stringify({status: "ok", base, screenshots: results.map(r => r.path), httpErrors, warnings: [...new Set(warnings)]}, null, 1));
} finally {
  await browser.close();
}
