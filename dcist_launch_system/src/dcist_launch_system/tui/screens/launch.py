"""Launch screen — configure and start experiment sessions."""

from __future__ import annotations

import threading

from rich.markup import escape
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (
    Checkbox,
    Footer,
    Input,
    Label,
    ProgressBar,
    RichLog,
    Rule,
    SelectionList,
)

from dcist_launch_system.fleet.launch_workflow import (
    TMUX_DIR,
    Confirmation,
    LaunchError,
    LaunchPlan,
    build_robot_commands,
    default_selection,
    dry_run_lines,
    list_sessions,
    run_launch,
)
from dcist_launch_system.fleet_helpers import (
    deploy_zenoh_config,
    generate_zenoh_endpoints,
)
from dcist_launch_system.tui.context import TuiContext
from dcist_launch_system.tui.screens.confirm import ConfirmScreen, TypeConfirmScreen
from dcist_launch_system.tui.screens.maps import PriorMapSelector


class LaunchScreen(ModalScreen):
    BINDINGS = [
        Binding("escape", "dismiss", "Back", priority=True),
        Binding("m", "preset_mapping", "Mapping (m)", priority=True),
        Binding("r", "preset_relocalize", "Relocalize (r)", priority=True),
        Binding("d", "dry_run", "Dry Run", priority=True),
        Binding("g", "do_launch", "Launch (g)", priority=True),
        Binding("z", "deploy_zenoh", "Deploy Zenoh (z)", priority=True),
        Binding("p", "browse_prior", "Refresh Priors (p)", priority=True),
        Binding("f2", "toggle_log_size", "Log Size [F2]", priority=True),
    ]

    DEFAULT_CSS = """
    LaunchScreen #robot_select {
        max-height: 6;
    }
    LaunchScreen #session_select {
        max-height: 12;
    }
    LaunchScreen #bs_select {
        max-height: 4;
    }
    LaunchScreen #bs_session_select {
        max-height: 6;
    }
    LaunchScreen #launch_prior_selector {
        height: auto;
    }
    LaunchScreen #prior_tree {
        height: 12;
        max-height: 16;
    }
    LaunchScreen #launch_log {
        min-height: 12;
    }
    """

    _LOG_SIZES = [12, 24, 40]

    def __init__(self, ctx: TuiContext, stop_zenohd=None, mode=None):
        super().__init__()
        self.ctx = ctx
        self._stop_zenohd = stop_zenohd
        self._launching = False
        self._mode = mode  # None=manual, "mapping", "relocalize"
        self._log_size_idx = 0

    def compose(self) -> ComposeResult:
        robots = [
            (m["name"], m["name"], m.get("online", False))
            for m in self.ctx.runtime_config.values()
            if m["role"] == "robot"
        ]
        base_stations = [
            (
                m["name"],
                f"{m['name']} (local)" if m["ip"] in self.ctx._local_ips else m["name"],
                m.get("online", False) or m["ip"] in self.ctx._local_ips,
            )
            for m in self.ctx.runtime_config.values()
            if m["role"] == "base_station"
        ]
        self._all_sessions = list_sessions(TMUX_DIR)

        yield VerticalScroll(
            Label(
                "[bold]Launch[/] — m=mapping, r=relocalize, g=launch, d=dry run, p=refresh priors"
            ),
            Rule(),
            Label("Select robots:"),
            SelectionList(*robots, id="robot_select"),
            Rule(),
            Label("Robot session:"),
            Input(placeholder="Filter sessions...", id="session_search"),
            SelectionList(*[(s, s) for s in self._all_sessions], id="session_select"),
            Rule(),
            Label("Base station (optional):"),
            SelectionList(*base_stations, id="bs_select"),
            Label("Base station session:"),
            SelectionList(
                *[(s, s) for s in self._all_sessions if "base_station" in s],
                id="bs_session_select",
            ),
            Rule(),
            Label(
                "Experiment (saved to ~/adt4_output/<name>/, e.g. 03252026_building_test):"
            ),
            Input(placeholder="date_description", id="experiment_input"),
            Checkbox("Record topics (--record)", id="record_checkbox"),
            Rule(),
            PriorMapSelector(self.ctx, id="launch_prior_selector"),
            Rule(),
            Vertical(id="launch_progress_area"),
            RichLog(id="launch_log", highlight=True, markup=True, max_lines=100),
        )
        yield Footer()

    def on_mount(self):
        if self._mode:
            self._apply_preset(self._mode, update_robots=True)

    def action_preset_mapping(self):
        self._mode = "mapping"
        # Save to app so re-opening preserves mode
        try:
            self.app._last_launch_mode = "mapping"
        except Exception:
            pass
        self._apply_preset("mapping")

    def action_preset_relocalize(self):
        self._mode = "relocalize"
        try:
            self.app._last_launch_mode = "relocalize"
        except Exception:
            pass
        self._apply_preset("relocalize")

    def _apply_preset(self, mode, update_robots=False):
        """Auto-populate selections from fleet_defaults in topology.

        update_robots: if True, reset robot selection to all online robots
        (only done on initial mount, not when the user switches mode).
        """
        log = self.query_one("#launch_log", RichLog)
        log.write(f"[bold cyan]Auto-populating for {mode} mode...[/]")

        robot_sel = self.query_one("#robot_select", SelectionList)
        preset = default_selection(
            self.ctx.topo,
            self.ctx.runtime_config,
            mode,
            None if update_robots else list(robot_sel.selected),
            self.ctx._local_ips,
        )
        # Select online robots (only on initial mount)
        if update_robots:
            robot_sel.deselect_all()
            for name in preset["robots"]:
                robot_sel.select(name)

        target_session = preset["session"]
        if target_session:
            sess_sel = self.query_one("#session_select", SelectionList)
            sess_sel.deselect_all()
            sess_sel.select(target_session)
            log.write(f"  Robot session: [cyan]{target_session}[/]")
            # Scroll to selected item
            try:
                idx = self._all_sessions.index(target_session)
                sess_sel.scroll_to_center(sess_sel.get_option_at_index(idx))
            except Exception:
                pass

        # Select local base station
        bs_sel = self.query_one("#bs_select", SelectionList)
        bs_sel.deselect_all()
        if preset["base_station"]:
            bs_sel.select(preset["base_station"])
            log.write(f"  Base station: [cyan]{preset['base_station']} (local)[/]")

        # Select BS session
        bs_sess_sel = self.query_one("#bs_session_select", SelectionList)
        bs_sess_sel.deselect_all()
        if preset["bs_session"]:
            bs_sess_sel.select(preset["bs_session"])
            log.write(f"  BS session: [cyan]{preset['bs_session']}[/]")

        if mode == "relocalize":
            log.write(
                "[yellow]Relocalize mode: prior map is required. Select one below or press p to refresh.[/]"
            )
        log.write("[dim]Review selections, then g=launch, d=dry run.[/]")

    def on_input_changed(self, event: Input.Changed):
        if event.input.id == "session_search":
            query = event.value.strip()
            sel = self.query_one("#session_select", SelectionList)
            sel.clear_options()
            for s in self._all_sessions:
                if self.ctx.fuzzy_match(query, s):
                    sel.add_option((s, s))

    def _get_selected_session(self):
        sel = self.query_one("#session_select", SelectionList)
        selected = list(sel.selected)
        return selected[0] if selected else ""

    def action_browse_prior(self):
        log = self.query_one("#launch_log", RichLog)
        self.query_one("#launch_prior_selector", PriorMapSelector).refresh_priors(log)

    def action_deploy_zenoh(self):
        """Deploy zenoh config to base station (robots just listen)."""
        log = self.query_one("#launch_log", RichLog)

        # Get selected robots (for endpoint scoping)
        robot_sel = self.query_one("#robot_select", SelectionList)
        robot_names = list(robot_sel.selected)

        # Only deploy to base station
        bs_sel = self.query_one("#bs_select", SelectionList)
        bs_names = list(bs_sel.selected)
        if not bs_names or bs_names[0] not in self.ctx.runtime_config:
            log.write("[yellow]No base station selected.[/]")
            return

        targets = [self.ctx.runtime_config[bs_names[0]]]
        log.write(f"[bold]Deploying zenoh config to {targets[0]['name']}...[/]")
        log.write(f"  [dim]Endpoints → robots: {', '.join(robot_names)}[/]")

        def do_deploy():
            results = []
            for m in targets:
                try:
                    endpoints = generate_zenoh_endpoints(
                        self.ctx.topo,
                        self.ctx.active_network,
                        m["name"],
                        robot_filter=robot_names,
                    )
                    ok, msg = deploy_zenoh_config(m["user"], m["ip"], endpoints)
                    results.append((m["name"], ok, msg, len(endpoints)))
                except KeyError:
                    results.append(
                        (
                            m["name"],
                            False,
                            f"no address on {self.ctx.active_network}",
                            0,
                        )
                    )
            return results

        def on_done(results):
            for name, ok, msg, n_ep in results:
                color = "green" if ok else "red"
                log.write(f"  [{color}]{name}: {msg} ({n_ep} endpoint(s))[/{color}]")
            log.write("[bold]Zenoh deploy complete.[/]")

        def bg():
            results = do_deploy()
            self.app.call_from_thread(on_done, results)

        threading.Thread(target=bg, daemon=True).start()

    def action_toggle_log_size(self):
        self._log_size_idx = (self._log_size_idx + 1) % len(self._LOG_SIZES)
        new_h = self._LOG_SIZES[self._log_size_idx]
        log = self.query_one("#launch_log", RichLog)
        log.styles.min_height = new_h

    def _current_plan(self) -> LaunchPlan:
        """Snapshot widget selections into a LaunchPlan (call on the UI thread)."""
        prior_path, prior_source = self.query_one(
            "#launch_prior_selector", PriorMapSelector
        ).get_prior_map()
        bs_selected = list(self.query_one("#bs_select", SelectionList).selected)
        bs_sess_selected = list(
            self.query_one("#bs_session_select", SelectionList).selected
        )
        return LaunchPlan(
            robots=list(self.query_one("#robot_select", SelectionList).selected),
            session=self._get_selected_session(),
            experiment=self.query_one("#experiment_input", Input).value.strip(),
            mode=self._mode,
            prior_path=prior_path or "",
            prior_source=prior_source if prior_path else None,
            record=self.query_one("#record_checkbox", Checkbox).value,
            base_station=bs_selected[0] if bs_selected else None,
            bs_session=bs_sess_selected[0] if bs_sess_selected else None,
        )

    def action_dry_run(self):
        log = self.query_one("#launch_log", RichLog)
        try:
            lines = dry_run_lines(
                self._current_plan(), self.ctx.runtime_config, self.ctx.output_root
            )
        except LaunchError as e:
            log.write(f"[yellow]{e}[/]")
            return
        log.write("[bold cyan]Dry run:[/]")
        for line in lines:
            log.write(f"  {line}")

    def _update_launch_progress(self, name, pct):
        try:
            bar = self.query_one(f"#lpb_{name}", ProgressBar)
            bar.update(progress=pct)
        except Exception:
            pass

    def action_do_launch(self):
        if self._launching:
            return
        log = self.query_one("#launch_log", RichLog)
        plan = self._current_plan()
        try:
            build_robot_commands(plan, self.ctx.runtime_config, self.ctx.output_root)
        except LaunchError as e:
            log.write(f"[yellow]{e}[/]")
            return

        self._launching = True
        log.write(f"[dim]robots = {plan.robots}, session = {plan.session}[/]")
        log.write(
            f"[dim]base station = {plan.base_station}, bs session = {plan.bs_session}[/]"
        )

        # Set up progress bars for prior transfers if needed
        area = self.query_one("#launch_progress_area", Vertical)
        area.remove_children()
        if plan.prior_path and plan.prior_source:
            targets = [r for r in plan.robots if r != plan.prior_source]
            if targets:
                area.mount(Label("[bold]Prior map transfer progress:[/]"))
                for name in targets:
                    area.mount(
                        Vertical(
                            Label(f"  {name}:"),
                            ProgressBar(total=100, show_eta=False, id=f"lpb_{name}"),
                        )
                    )

        callbacks = _TuiLaunchCallbacks(self.app, log, self, self._stop_zenohd)

        def do_launch():
            try:
                run_launch(
                    plan,
                    topo=self.ctx.topo,
                    active_network=self.ctx.active_network,
                    runtime_config=self.ctx.runtime_config,
                    output_root=self.ctx.output_root,
                    callbacks=callbacks,
                )
            finally:
                self._launching = False

        threading.Thread(target=do_launch, daemon=True).start()


_LEVEL_MARKUP = {
    "heading": "[bold]{}[/]",
    "info": "  {}",
    "dim": "  [dim]{}[/]",
    "ok": "  [green]{}[/]",
    "warn": "[yellow]{}[/]",
    "error": "  [red]{}[/]",
}


class _TuiLaunchCallbacks:
    """Adapts the launch workflow's callbacks to Textual (called from a worker thread)."""

    def __init__(self, app, log, screen, stop_zenohd):
        self._app = app
        self._log = log
        self._screen = screen
        self._stop_zenohd = stop_zenohd

    def log(self, level, text):
        line = _LEVEL_MARKUP.get(level, "{}").format(escape(text))
        self._app.call_from_thread(self._log.write, line)

    def progress(self, machine, pct):
        self._app.call_from_thread(self._screen._update_launch_progress, machine, pct)

    def confirm(self, request: Confirmation):
        decision = [None]
        ev = threading.Event()

        def ask():
            def on_dismiss(result):
                decision[0] = result
                ev.set()

            msg = escape(request.message)
            if request.required_text:
                screen = TypeConfirmScreen(msg, required_text=request.required_text)
            else:
                screen = ConfirmScreen(
                    msg,
                    yes_label=f"{request.yes_label} (y)",
                    no_label=f"{request.no_label} (n)",
                )
            self._app.push_screen(screen, on_dismiss)

        self._app.call_from_thread(ask)
        ev.wait(timeout=120)
        return decision[0]

    def local_zenohd_running(self):
        ctx = self._screen.ctx
        return bool(ctx._fleet_zenoh["proc"])

    def stop_local_zenohd(self):
        if self._stop_zenohd:
            self._stop_zenohd()


# ---- Bandwidth Selection Screen ----
