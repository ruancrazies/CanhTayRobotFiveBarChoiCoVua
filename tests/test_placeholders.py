"""Placeholders stay detectable until phase 5 (SPEC 7.2, "Rules clarified in phase 2").

Grasp angles, reserve slot contents and the capture/reserve/park layout are listed under ``assumed`` in
config/settings.yaml (not confirmed on the real hardware), so every value that is not explicitly confirmed is a
PLACEHOLDER. The phase 3 Robot Commander will use
``MovePlanner.placeholders()`` to guard a real serial port. The guard itself is NOT built here; these
tests prove that the information it needs is correct and cannot be lost by accident.
"""

from __future__ import annotations

import builtins
import dataclasses
from pathlib import Path

import chess
import pytest
import yaml

from desktop_app import console_runner
from desktop_app.core.move_planner import GraspProfiles, MovePlanner, ReserveSlot, reserve_slots_from_settings
from desktop_app.kinematics.dimension import Zones
from desktop_app.utils.settings import DEFAULT_SETTINGS_PATH, Settings, get_settings, load_settings
from phase2_support import (
    TEST_GRASP,
    TEST_RESERVE,
    make_geometry,
    make_planner,
    make_workspace,
    make_zones,
)

E2E4 = chess.Move.from_uci("e2e4")
Q, R, B, N = chess.QUEEN, chess.ROOK, chess.BISHOP, chess.KNIGHT


def planner_with(grasp: GraspProfiles, reserve: tuple[object, ...], *, zones_placeholder: bool = True) -> MovePlanner:
    """A planner whose zone layout is a placeholder unless ``zones_placeholder=False``."""
    return MovePlanner(
        make_workspace(), make_geometry(), make_zones(reserve_slots=len(reserve), is_placeholder=zones_placeholder),
        grasp, reserve_pieces=reserve,  # type: ignore[arg-type]
    )


def confirmed_reserve() -> tuple[ReserveSlot, ...]:
    return tuple(ReserveSlot(piece, is_placeholder=False) for piece in TEST_RESERVE)


def real_grasp(_unused: object = None) -> GraspProfiles:
    """A confirmed grasp profile (what the loader returns once the grasp keys leave ``assumed``)."""
    return GraspProfiles.from_dict(TEST_GRASP, placeholder=False)


def settings_without_assumed(tmp_path: Path, *prefixes: str) -> Settings:
    """settings.yaml with every key under the given prefixes removed from ``assumed`` (= confirmed on the hardware)."""
    data = yaml.safe_load(DEFAULT_SETTINGS_PATH.read_text(encoding="utf-8"))
    data["assumed"] = [k for k in data["assumed"] if not any(k == p or k.startswith(p + ".") for p in prefixes)]
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return load_settings(path, overlay=False)


# --------------------------------------------------------------------------- #
# Grasp profiles
# --------------------------------------------------------------------------- #


def test_a_grasp_profile_is_a_placeholder_unless_settings_yaml_confirms_it(tmp_path: Path) -> None:
    assert GraspProfiles.from_dict(TEST_GRASP).is_placeholder  # fail-safe default
    assert GraspProfiles.from_dict(TEST_GRASP, placeholder=True).is_placeholder
    assert not GraspProfiles.from_dict(TEST_GRASP, placeholder=False).is_placeholder
    assert GraspProfiles(open_deg=1.0, close_deg={}).is_placeholder  # even a hand-built one
    assert not real_grasp().is_placeholder
    assert GraspProfiles.from_settings().is_placeholder  # the shipped angles are assumed
    confirmed = settings_without_assumed(tmp_path, "grasp")
    assert not GraspProfiles.from_settings(confirmed).is_placeholder


def test_a_grasp_profile_cannot_be_turned_into_a_real_one_by_assignment() -> None:
    profile = GraspProfiles.from_dict(TEST_GRASP)
    with pytest.raises(dataclasses.FrozenInstanceError):
        profile.is_placeholder = False  # type: ignore[misc]


def test_the_shipped_grasp_angles_are_listed_as_assumed() -> None:
    assert get_settings().assumed_under("grasp"), "real angles must come from the hardware owner (phase 5)"


# --------------------------------------------------------------------------- #
# Reserve slots
# --------------------------------------------------------------------------- #


def test_a_reserve_slot_is_a_placeholder_unless_it_says_it_is_confirmed() -> None:
    assert ReserveSlot(Q).is_placeholder
    assert ReserveSlot(Q, is_placeholder=True).is_placeholder
    assert not ReserveSlot(Q, is_placeholder=False).is_placeholder
    with pytest.raises(dataclasses.FrozenInstanceError):
        ReserveSlot(Q).is_placeholder = False  # type: ignore[misc]


def test_reserve_contents_follow_settings_yaml(tmp_path: Path) -> None:
    shipped = reserve_slots_from_settings()
    assert [slot.piece_type for slot in shipped] == [chess.QUEEN, chess.ROOK, chess.BISHOP, chess.KNIGHT]
    assert all(slot.is_placeholder for slot in shipped)
    confirmed = reserve_slots_from_settings(settings_without_assumed(tmp_path, "layout.reserve_pieces"))
    assert not any(slot.is_placeholder for slot in confirmed)


def test_bare_piece_types_count_as_placeholder_slots() -> None:
    planner = make_planner()  # reserve_pieces=TEST_RESERVE, plain piece types
    assert planner.reserve_pieces == TEST_RESERVE
    assert [slot.piece_type for slot in planner.reserve_slots] == list(TEST_RESERVE)
    assert all(slot.is_placeholder for slot in planner.reserve_slots)


# --------------------------------------------------------------------------- #
# MovePlanner.placeholders(): what the phase 3 Robot Commander will ask
# --------------------------------------------------------------------------- #


def test_everything_is_listed_while_nothing_is_confirmed() -> None:
    planner = make_planner()
    assert planner.placeholders() == (
        "grasp_profiles", "zones", "reserve_slot[0]", "reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]",
    )
    assert planner.has_placeholders


def test_only_the_reserve_slots_are_listed_once_the_grasp_angles_and_zones_are_confirmed() -> None:
    planner = planner_with(real_grasp(), TEST_RESERVE, zones_placeholder=False)
    assert planner.placeholders() == ("reserve_slot[0]", "reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]")
    assert planner.has_placeholders


def test_only_the_grasp_profile_is_listed_once_the_reserve_slots_and_zones_are_confirmed() -> None:
    planner = planner_with(GraspProfiles.from_dict(TEST_GRASP), confirmed_reserve(), zones_placeholder=False)
    assert planner.placeholders() == ("grasp_profiles",)
    assert planner.has_placeholders


def test_only_the_zone_layout_is_listed_once_the_grasp_angles_and_reserve_slots_are_confirmed() -> None:
    planner = planner_with(real_grasp(), confirmed_reserve(), zones_placeholder=True)
    assert planner.placeholders() == ("zones",)
    assert planner.has_placeholders


def test_the_zone_layout_is_listed_between_the_grasp_profile_and_the_reserve_slots() -> None:
    names = make_planner().placeholders()
    assert names.index("grasp_profiles") < names.index("zones") < names.index("reserve_slot[0]")


def test_one_unconfirmed_slot_is_enough_to_keep_the_planner_flagged() -> None:
    reserve = (ReserveSlot(Q, False), ReserveSlot(R, False), ReserveSlot(B, True), ReserveSlot(N, False))
    planner = planner_with(real_grasp(), reserve, zones_placeholder=False)
    assert planner.placeholders() == ("reserve_slot[2]",)
    assert planner.has_placeholders


def test_nothing_is_listed_only_when_every_value_is_confirmed() -> None:
    planner = planner_with(real_grasp(), confirmed_reserve(), zones_placeholder=False)
    assert planner.placeholders() == ()
    assert not planner.has_placeholders
    assert planner.grasp is not None and not planner.grasp.is_placeholder


def test_mixed_input_keeps_each_slot_flag_in_zone_order() -> None:
    reserve = (ReserveSlot(Q, False), R, ReserveSlot(B, True), N)  # a bare piece type is a placeholder
    planner = planner_with(GraspProfiles.from_dict(TEST_GRASP, placeholder=False), reserve, zones_placeholder=False)
    assert planner.reserve_pieces == (Q, R, B, N)
    assert planner.placeholders() == ("reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]")


def test_the_flag_is_information_only_the_chain_is_the_same() -> None:
    placeholder = make_planner()
    confirmed = planner_with(real_grasp(), confirmed_reserve(), zones_placeholder=False)
    board = chess.Board()
    assert placeholder.plan(board, E2E4).describe() == confirmed.plan(board, E2E4).describe()
    assert placeholder.has_placeholders and not confirmed.has_placeholders


def test_a_planner_without_a_reserve_zone_has_no_reserve_placeholders() -> None:
    planner = MovePlanner(
        make_workspace(), make_geometry(), make_zones(reserve_slots=None, is_placeholder=False), real_grasp()
    )
    assert planner.reserve_slots == () and planner.placeholders() == ()


# --------------------------------------------------------------------------- #
# The capture/reserve/park layout
# --------------------------------------------------------------------------- #


def test_the_layout_of_settings_yaml_is_listed_as_a_placeholder_while_assumed(tmp_path: Path) -> None:
    """``Zones.from_settings`` is a placeholder while the layout keys are in ``assumed`` and confirmed once they leave it."""
    proposed = Zones.from_settings(make_geometry(), make_workspace())
    assert proposed.is_placeholder
    confirmed = settings_without_assumed(tmp_path, "layout.capture_slots", "layout.reserve_slots", "layout.park")
    assert not Zones.from_settings(make_geometry(), make_workspace(), confirmed).is_placeholder
    planner = MovePlanner(
        make_workspace(), make_geometry(), proposed, real_grasp(), reserve_pieces=confirmed_reserve()
    )
    assert planner.placeholders() == ("zones",)


def test_an_incomplete_zone_layout_is_still_flagged() -> None:
    layout = Zones(make_geometry(), make_workspace())  # nothing defined yet, so nothing is confirmed either
    planner = MovePlanner(make_workspace(), make_geometry(), layout, real_grasp())
    assert planner.placeholders() == ("zones",)


# --------------------------------------------------------------------------- #
# The console runner's demo values are placeholders, and says so
# --------------------------------------------------------------------------- #


def test_the_values_the_runner_uses_are_all_placeholders_while_assumed() -> None:
    planner = MovePlanner(
        make_workspace(), make_geometry(), Zones.from_settings(make_geometry(), make_workspace()),
        GraspProfiles.from_settings(), reserve_pieces=reserve_slots_from_settings(),
    )
    assert planner.placeholders() == (
        "grasp_profiles", "zones", "reserve_slot[0]", "reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]",
    )


def test_the_command_line_lists_what_is_still_a_placeholder(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(builtins, "input", lambda _prompt="": "quit")
    assert console_runner.main(["--demo-placeholders", "--data-dir", str(tmp_path), "--new"]) == 0
    printed = capsys.readouterr().out
    assert "Still placeholders" in printed and "grasp_profiles" in printed and "reserve_slot[3]" in printed
    assert "grasp_profiles, zones, reserve_slot[0]" in printed  # the zone layout is listed too
