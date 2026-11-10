"""The chat's auto-send mode tells the model its roster is the online robots and
that plans go out at once (the frontend sends them as PDDL goals)."""


def test_auto_send_context_names_the_mode():
    from adt4_console.mission.llm_planner import context_text

    roster = [{"id": "gauss", "kind": "husky", "start": None, "binding": "gauss"}]
    on = context_text({"roster": roster, "auto_send": True})
    assert "Auto-send is ON" in on and '"kind":"husky"' in on
    assert "Auto-send" not in context_text({"roster": roster})
