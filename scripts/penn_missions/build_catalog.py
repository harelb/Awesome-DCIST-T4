"""Build the mission catalog page: python build_catalog.py missions.yaml MISSIONS_DIR VIDEOS_DIR OUT_DIR

OUT_DIR gets index.html plus media/<id>/{overhead.mp4,final.jpg,goal_*.jpg} referenced relatively."""
import html, json, pathlib, shutil, subprocess, sys
import yaml

cat = yaml.safe_load(open(sys.argv[1]))
MD, VD, OUT = (pathlib.Path(p) for p in sys.argv[2:5])
media = OUT / "media"
shutil.rmtree(media, ignore_errors=True)
media.mkdir(parents=True)
E = html.escape
ROBOT = {"euclid": "rb-euclid", "hamilton": "rb-hamilton", "gauss": "rb-gauss"}
ICON = {"goto": "go to", "inspect": "inspect", "pick": "pick up", "place": "put down", "drag": "grab to drag"}


def jpg(src, dst, w=480):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(src), "-vf", f"scale='min({w},iw)':-2", "-q:v", "4", str(dst)], check=True)


def task_line(t, objs):
    typ = t["type"]
    tgt = t["target"]
    tgt = tgt if isinstance(tgt, str) else f"({tgt[0]:.0f}, {tgt[1]:.0f})"
    if typ == "place":
        what = t["params"].get("object", "")
        verb = "drag " + what + " to" if t["params"].get("dragged") else "put " + what + " down at"
        return f"{verb} {tgt}"
    note = t.get("note", "")
    if "explore" in note and "(" in note:
        return f"explore waypoint {tgt} <span class='muted'>{E(note[note.rfind('('):])}</span>"
    return f"{ICON.get(typ, typ)} {tgt}"


cards, nav = [], []
total_drag = 0
for m in cat["missions"]:
    mid = m["id"]
    d = MD / mid
    if not (d / "plan.json").exists():
        continue
    plan = json.loads((d / "plan.json").read_text())
    pv = json.loads((d / "preview.json").read_text())
    res = json.loads((d / "result.json").read_text())
    mission = plan["mission"]
    out = media / mid
    out.mkdir()
    vid = VD / mid / "overhead.mp4"
    video_html = "<div class='novideo'>video not recorded</div>"
    if vid.exists():
        shutil.copy(vid, out / "overhead.mp4")
        poster = ""
        if (VD / mid / "final.png").exists():
            jpg(VD / mid / "final.png", out / "final.jpg", 1200)
            poster = f" poster='media/{mid}/final.jpg'"
        video_html = f"<video controls muted playsinline preload='none'{poster} src='media/{mid}/overhead.mp4'></video>"
    # per-robot tasks
    by = {}
    for t in mission["tasks"]:
        by.setdefault(t["agent"], []).append(t)
    drags = sum(1 for t in mission["tasks"] if t["type"] == "drag")
    total_drag += drags
    robots_html = []
    for r in ["euclid", "hamilton", "gauss"]:
        ts = by.get(r, [])
        km = pv["driven_m"].get(r, 0.0)
        steps = []
        explore = [t for t in ts if t["type"] == "goto" and "explore" in (t.get("note") or "")]
        shown = 0
        for t in ts:
            if t in explore[1:-1]:
                continue
            steps.append(f"<li>{task_line(t, None)}</li>")
            shown += 1
            if explore and t is explore[0]:
                if len(explore) > 2:
                    steps.append(f"<li class='muted'>… {len(explore) - 2} more coverage waypoints …</li>")
        robots_html.append(
            f"<div class='robot'><div class='rhead'><span class='dot {ROBOT[r]}'></span><b>{r}</b>"
            f"<span class='kind'>{'Husky, no arm' if r == 'gauss' else 'Spot + arm'}</span>"
            f"<span class='dist'>{km:.0f} m</span></div><ol>{''.join(steps) or '<li class=muted>no tasks</li>'}</ol></div>")
    # goal images
    imgs = []
    for p in sorted((d / "goal_images").glob("*.jpg")):
        if p.stem.endswith("_context"):
            continue
        dst = out / ("goal_" + p.name)
        jpg(p, dst, 360)
        parts = p.stem.split("_", 2)
        robot, sym = (parts[1], parts[2]) if len(parts) == 3 else ("", p.stem)
        imgs.append(f"<figure><img loading='lazy' src='media/{mid}/{dst.name}' alt='goal {E(sym)} for {E(robot)}'>"
                    f"<figcaption><span class='dot {ROBOT.get(robot, '')}'></span>{E(robot)} · {E(sym)}</figcaption></figure>")
    if len(imgs) > 9:
        imgs = imgs[:8] + [f"<figure class='more'><span>+{len(imgs) - 8} more waypoint views</span></figure>"]
    cons = m.get("constraints") or []
    cons_html = "".join(f"<code>{c['predicate']} {' → '.join(c['symbols'])}</code> " for c in cons)
    chat_goal = res.get("chat_goal")
    chat_html = (f"<pre>{E(chat_goal)}</pre>" if chat_goal else "<p class='muted'>The chat did not return a goal for this command (see notes).</p>")
    mk = res.get("makespan_s") or 0
    engine = "GRSTAPS-X" if m["engine"] == "grstaps" else "Fast Downward"
    tags = "".join(f"<li>{E(s)}</li>" for s in m.get("shows", []))
    nav.append(f"<a href='#{mid.lower()}'><b>{mid}</b> {E(m['title'])}</a>")
    if mid == "M11":
        cards.append('<h2 class="divider">Without drag: M11–M15</h2><p class="muted divider-note">The Spot drag skill is still being built, so these five use only what the robots do today. Spots carry cones and light bags with the arm; gauss visits, inspects, sweeps and blocks.</p>')
    cards.append(f"""
<section class="mission" id="{mid.lower()}">
  <header class="mhead">
    <div class="mid">{mid}</div>
    <div class="mtitle"><h2>{E(m['title'])}</h2>
      <div class="meta"><span class="engine e-{m['engine']}">{engine}</span>
      <span>≈ {mk/60:.1f} min</span><span>{sum(pv['driven_m'].values()):.0f} m driven</span>{'<span class="drag">drag ×%d</span>' % drags if drags else ''}{'<span class="nodrag">no drag</span>' if m.get('nodrag') else ''}</div></div>
  </header>
  <blockquote class="command">“{E(m['text'])}”</blockquote>
  <div class="grid">
    <div class="media">{video_html}</div>
    <div class="side">
      <h3>What it exercises</h3><ul class="tags">{tags}</ul>
      <h3>Field setup</h3><p>{E(m.get('setup', ''))}</p>
    </div>
  </div>
  <div class="robots">{''.join(robots_html)}</div>
  <h3>Goal images</h3>
  <div class="goals">{''.join(imgs)}</div>
  <details><summary>Planner goal and the chat's interpretation</summary>
    <h4>Canonical goal ({engine})</h4><pre>{E(m['goal'])}</pre>{('<p>Constraints: ' + cons_html + '</p>') if cons else ''}
    <h4>gpt-6-luna's goal from the command</h4>{chat_html}
    <p class="muted">Console mission id <code>{res['mission_id']}</code> on the dev console (:8099).</p>
  </details>
</section>""")

start = cat["starts"]
page = f"""<title>Pennovation Mission Suite</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Semi+Condensed:wght@500;600;700&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
/* Layout: field-binder pages. A sticky mission index, then one sheet per mission: command, video, robot columns, goal photos. */
:root {{
  --bg: #f3f4f1; --sheet: #ffffff; --ink: #1d2320; --muted: #5d6762; --line: #d9ddd8;
  --cone: #e8590c; --cone-soft: #fde7da; --grstaps: #3b5bdb; --fd: #2b8a3e;
  --euclid: #1c9fc4; --hamilton: #d6488f; --gauss: #b8730f;
  --display: "Barlow Semi Condensed", "Arial Narrow", sans-serif;
  --body: "IBM Plex Sans", system-ui, sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #141816; --sheet: #1c211e; --ink: #e6ebe7; --muted: #9aa49e; --line: #313833;
  --cone: #ff7a33; --cone-soft: #3a2418; --grstaps: #8ea6ff; --fd: #69db7c;
  --euclid: #4cc3e6; --hamilton: #f07ab3; --gauss: #e3a33f; color-scheme: dark }} }}
:root[data-theme="dark"] {{
  --bg: #141816; --sheet: #1c211e; --ink: #e6ebe7; --muted: #9aa49e; --line: #313833;
  --cone: #ff7a33; --cone-soft: #3a2418; --grstaps: #8ea6ff; --fd: #69db7c;
  --euclid: #4cc3e6; --hamilton: #f07ab3; --gauss: #e3a33f; color-scheme: dark }}
* {{ box-sizing: border-box }}
body {{ background: var(--bg); color: var(--ink); font: 15px/1.55 var(--body);
  padding-left: max(16px, env(safe-area-inset-left, 0px)); padding-right: max(16px, env(safe-area-inset-right, 0px)); padding-block: 0 48px }}
.wrap {{ max-width: 1180px; margin: 0 auto; display: grid; gap: 28px }}
h1, h2, h3 {{ font-family: var(--display); text-wrap: balance; margin: 0 }}
h1 {{ font-size: clamp(2rem, 5vw, 3.1rem); line-height: 1.05; letter-spacing: .01em }}
h2 {{ font-size: 1.7rem; line-height: 1.1 }}
h3 {{ font-size: .82rem; text-transform: uppercase; letter-spacing: .09em; color: var(--muted); margin-top: 6px }}
h4 {{ margin: 12px 0 4px; font-size: .9rem }}
.intro {{ padding-top: 36px; display: grid; gap: 14px }}
.intro p {{ max-width: 70ch; margin: 0 }}
.kicker {{ font-family: var(--mono); font-size: .8rem; color: var(--cone); letter-spacing: .06em; text-transform: uppercase }}
.facts {{ display: flex; flex-wrap: wrap; gap: 10px 22px; font-size: .9rem; color: var(--muted) }}
.facts b {{ color: var(--ink) }}
.dot {{ display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; vertical-align: baseline }}
.rb-euclid {{ background: var(--euclid) }} .rb-hamilton {{ background: var(--hamilton) }} .rb-gauss {{ background: var(--gauss) }}
nav.index {{ position: sticky; top: env(safe-area-inset-top, 0px); z-index: 5; background: var(--bg); border-bottom: 1px solid var(--line);
  display: flex; gap: 6px; overflow-x: auto; padding: 10px 0; scrollbar-width: thin }}
body::before {{ content: ""; position: fixed; inset: 0 0 auto; height: env(safe-area-inset-top, 0px); z-index: 10; background: var(--bg) }}
nav.index a {{ flex: none; text-decoration: none; color: var(--ink); font-size: .85rem; padding: 5px 10px; border: 1px solid var(--line); border-radius: 999px; background: var(--sheet) }}
nav.index a b {{ font-family: var(--mono); color: var(--cone); font-weight: 500 }}
nav.index a:hover, nav.index a:focus-visible {{ border-color: var(--cone); outline: none }}
.mission {{ background: var(--sheet); border: 1px solid var(--line); border-radius: 6px; padding: 22px clamp(14px, 3vw, 28px); display: grid; gap: 16px; scroll-margin-top: 64px }}
.mhead {{ display: flex; gap: 16px; align-items: center }}
.mid {{ font-family: var(--display); font-weight: 700; font-size: 1.6rem; color: var(--cone); background: var(--cone-soft);
  border-radius: 4px; padding: 4px 10px; flex: none }}
.mtitle {{ min-width: 0; display: grid; gap: 4px }}
.meta {{ display: flex; flex-wrap: wrap; gap: 6px 14px; font-size: .85rem; color: var(--muted); font-variant-numeric: tabular-nums }}
.engine {{ font-weight: 600 }} .e-grstaps {{ color: var(--grstaps) }} .e-fast_downward {{ color: var(--fd) }}
.drag {{ color: var(--cone); font-weight: 600 }} .nodrag {{ color: var(--fd); font-weight: 600 }}
.command {{ margin: 0; font-size: 1.12rem; line-height: 1.5; border-left: 3px solid var(--cone); padding: 2px 0 2px 14px; max-width: 80ch }}
.grid {{ display: grid; grid-template-columns: minmax(0, 1.9fr) minmax(0, 1fr); gap: 20px }}
.media video {{ width: 100%; max-width: 100%; border-radius: 4px; background: #000; display: block; aspect-ratio: 16 / 10 }}
.novideo {{ aspect-ratio: 16/10; display: grid; place-items: center; color: var(--muted); border: 1px dashed var(--line) }}
.side {{ display: grid; gap: 8px; align-content: start; min-width: 0 }}
.side p {{ margin: 0 }}
.tags {{ margin: 0; padding-left: 18px; display: grid; gap: 3px }}
.robots {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 14px }}
.robot {{ border-top: 2px solid var(--line); padding-top: 8px; min-width: 0 }}
.rhead {{ display: flex; align-items: center; gap: 4px; flex-wrap: wrap }}
.kind {{ font-size: .78rem; color: var(--muted); margin-left: 6px }}
.dist {{ margin-left: auto; font-family: var(--mono); font-size: .82rem; color: var(--muted) }}
.robot ol {{ margin: 6px 0 0; padding-left: 20px; font-size: .9rem; display: grid; gap: 2px }}
.muted {{ color: var(--muted) }}
.goals {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 10px }}
.goals figure {{ margin: 0; display: grid; gap: 4px }}
.goals img {{ width: 100%; aspect-ratio: 4 / 3; object-fit: cover; border-radius: 3px; background: var(--line) }}
.goals figcaption {{ font-family: var(--mono); font-size: .75rem; color: var(--muted) }}
.goals .more {{ display: grid; place-items: center; aspect-ratio: 4/3; border: 1px dashed var(--line); color: var(--muted); font-size: .85rem; text-align: center; padding: 8px }}
details {{ border-top: 1px solid var(--line); padding-top: 10px }}
summary {{ cursor: pointer; font-weight: 600 }}
summary:focus-visible {{ outline: 2px solid var(--cone); outline-offset: 2px }}
pre, code {{ font-family: var(--mono); font-size: .82rem }}
pre {{ background: var(--bg); padding: 10px 12px; border-radius: 4px; overflow-x: auto; white-space: pre-wrap; word-break: break-word; margin: 0 }}
.divider {{ margin-top: 12px }} .divider-note {{ margin: -18px 0 0; max-width: 80ch }}
.notes {{ display: grid; gap: 8px }} .notes ul {{ margin: 0; padding-left: 20px; display: grid; gap: 4px; max-width: 90ch }}
@media (max-width: 820px) {{ .grid, .robots {{ grid-template-columns: minmax(0, 1fr) }} }}
@media (prefers-reduced-motion: reduce) {{ * {{ scroll-behavior: auto !important }} }}
html {{ scroll-behavior: smooth }}
</style>
<div class="wrap">
  <header class="intro">
    <div class="kicker">Pennovation · fused Penn scene graph · 15 missions</div>
    <h1>Pennovation Mission Suite</h1>
    <p>Fifteen natural-language missions for two Spots and a Husky. M11–M15 avoid the drag skill, which is still being built. Each mixes region association, Cypher lookups and open-set grounding. Five are planned with Fast Downward and five with GRSTAPS-X, which handles ordering, joint inspections and arm-only tasks. Each video is the console's overhead simulation of the planned mission. The goal images are what each robot should expect to see.</p>
    <div class="facts">
      <span><span class="dot rb-euclid"></span><b>euclid</b> Spot + arm, start ({start['euclid']['start'][0]:.0f}, {start['euclid']['start'][1]:.0f})</span>
      <span><span class="dot rb-hamilton"></span><b>hamilton</b> Spot + arm, start ({start['hamilton']['start'][0]:.0f}, {start['hamilton']['start'][1]:.0f})</span>
      <span><span class="dot rb-gauss"></span><b>gauss</b> Husky, no arm, start ({start['gauss']['start'][0]:.0f}, {start['gauss']['start'][1]:.0f})</span>
      <span>All start at the black shelter by the r0/r12 gate (place t10739)</span>
    </div>
  </header>
  <nav class="index" aria-label="Missions">{''.join(nav)}</nav>
  {''.join(cards)}
  <section class="mission notes" id="notes">
    <h2>Notes for the field day</h2>
    <ul>
      <li><b>Drag.</b> Heavy objects (bags, sandbags, boxes, trash cans) are dragged, not picked. Both planners decide this from the object's class. The robots receive <code>DRAG_START</code>, a follow path, then <code>DRAG_RELEASE</code>. The Spot executor still has to implement those two action types (spot_tools <code>drag_utils</code> on <code>pioneer_adelene</code> is the starting point).</li>
      <li><b>Husky in GRSTAPS-X.</b> Plans now include gauss as a robot without an arm, and the schedules order its tasks (M9: gauss only moves after the cone is placed). On the robots, the compiled plan does not yet make Spots wait on gauss or gauss wait on Spots. That needs the executor-side ordering you mentioned.</li>
      <li><b>Open-set objects.</b> O704 (white water tank), O705 and O706 (orange barriers) were grounded with SAM3 on the dev copy of the map. M7's bags are placed on the day: put two bags near (−40, −10), ground them with the chat's image search, then plan with the new symbols.</li>
      <li><b>Moved objects.</b> Cones and bags in the prior map may have moved since it was recorded. Before each run, check the targets in the goal images, or re-ground them with SAM3.</li>
      <li><b>Planning limits.</b> GRSTAPS-X solves up to about 20 tasks in its 60 s limit. Each explored region adds up to 15 coverage waypoints, so keep to one exploration per mission or raise <code>ADT4_GRSTAPS_TIMEOUT_S</code>. Fast Downward's region exploration visits every place in the region and does not scale to big regions.</li>
      <li><b>Allocation from language.</b> Neither planner can assign a relocation to a named robot. "Euclid carries X" is written as <code>(visited-object-by euclid X)</code> next to the <code>object-in-place</code>, which in practice makes euclid do it.</li>
    </ul>
  </section>
</div>
"""
(OUT / "index.html").write_text(page)
print("wrote", OUT / "index.html", "drags", total_drag)
