from dcist_sim_isaac.gt_capture import match_semantics
from dcist_sim_isaac.scenario import GtSemanticRule

PATHS = [
    "/World/Environment/SM_PaletteA_01",
    "/World/Environment/Forklift/body",
    "/World/Environment/floor",
]


def test_match_first_rule_wins():
    rules = [
        GtSemanticRule(match=".*Palette.*", semantic_class="pallet"),
        GtSemanticRule(match=".*SM_.*", semantic_class="prop"),
    ]
    out = match_semantics(PATHS, rules)
    assert out["/World/Environment/SM_PaletteA_01"] == "pallet"


def test_unmatched_paths_absent():
    rules = [GtSemanticRule(match=".*Forklift.*", semantic_class="forklift")]
    out = match_semantics(PATHS, rules)
    assert list(out) == ["/World/Environment/Forklift/body"]


def test_no_rules_empty():
    assert match_semantics(PATHS, []) == {}
