"""Placeholders stay detectable until phase 5 (SPEC 7.2, "Rules clarified in phase 2").

Grasp angles and reserve slot contents are [TO CONFIRM], so every value that is not explicitly
confirmed is a PLACEHOLDER. The phase 3 Robot Commander will refuse a real serial port while
``MovePlanner.placeholders()`` is not empty. The guard itself is NOT built here; these tests prove
that the information it needs is correct and cannot be lost by accident.
"""

from __future__ import annotations

import builtins
import dataclasses
import json
from pathlib import Path

import chess
import pytest

from desktop_app import console_runner
from desktop_app.core.move_planner import GraspProfiles, MovePlanner, ReserveSlot
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


def planner_with(grasp: GraspProfiles, reserve: tuple[object, ...]) -> MovePlanner:
    return MovePlanner(
        make_workspace(), make_geometry(), make_zones(reserve_slots=len(reserve)), grasp,
        reserve_pieces=reserve,  # type: ignore[arg-type]
    )


def real_grasp(tmp_path: Path) -> GraspProfiles:
    """A grasp profile the way a confirmed one arrives: loaded from a config file."""
    path = tmp_path / "grasp_profiles.json"
    path.write_text(json.dumps(TEST_GRASP), encoding="utf-8")
    return GraspProfiles.from_file(path)


# --------------------------------------------------------------------------- #
# Grasp profiles
# --------------------------------------------------------------------------- #


def test_a_grasp_profile_is_a_placeholder_unless_it_comes_from_the_config_file(tmp_path: Path) -> None:
    assert GraspProfiles.from_dict(TEST_GRASP).is_placeholder  # fail-safe default
    assert GraspProfiles.from_dict(TEST_GRASP, placeholder=True).is_placeholder
    assert not GraspProfiles.from_dict(TEST_GRASP, placeholder=False).is_placeholder
    assert GraspProfiles(open_deg=1.0, close_deg={}).is_placeholder  # even a hand-built one
    assert not real_grasp(tmp_path).is_placeholder


def test_a_grasp_profile_cannot_be_turned_into_a_real_one_by_assignment() -> None:
    profile = GraspProfiles.from_dict(TEST_GRASP)
    with pytest.raises(dataclasses.FrozenInstanceError):
        profile.is_placeholder = False  # type: ignore[misc]


def test_the_repository_ships_no_real_grasp_profile() -> None:
    from desktop_app.core.move_planner import DEFAULT_GRASP_PROFILES_PATH

    assert not DEFAULT_GRASP_PROFILES_PATH.exists(), "real angles must come from the hardware owner (phase 5)"


# --------------------------------------------------------------------------- #
# Reserve slots
# --------------------------------------------------------------------------- #


def test_a_reserve_slot_is_a_placeholder_unless_it_says_it_is_confirmed() -> None:
    assert ReserveSlot(Q).is_placeholder
    assert ReserveSlot(Q, is_placeholder=True).is_placeholder
    assert not ReserveSlot(Q, is_placeholder=False).is_placeholder
    with pytest.raises(dataclasses.FrozenInstanceError):
        ReserveSlot(Q).is_placeholder = False  # type: ignore[misc]


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
        "grasp_profiles", "reserve_slot[0]", "reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]",
    )
    assert planner.has_placeholders


def test_only_the_reserve_slots_are_listed_once_the_grasp_angles_are_confirmed(tmp_path: Path) -> None:
    planner = planner_with(real_grasp(tmp_path), TEST_RESERVE)
    assert planner.placeholders() == ("reserve_slot[0]", "reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]")
    assert planner.has_placeholders


def test_only_the_grasp_profile_is_listed_once_the_reserve_slots_are_confirmed() -> None:
    reserve = tuple(ReserveSlot(piece, is_placeholder=False) for piece in TEST_RESERVE)
    planner = planner_with(GraspProfiles.from_dict(TEST_GRASP), reserve)
    assert planner.placeholders() == ("grasp_profiles",)
    assert planner.has_placeholders


def test_one_unconfirmed_slot_is_enough_to_keep_the_planner_flagged(tmp_path: Path) -> None:
    reserve = (ReserveSlot(Q, False), ReserveSlot(R, False), ReserveSlot(B, True), ReserveSlot(N, False))
    planner = planner_with(real_grasp(tmp_path), reserve)
    assert planner.placeholders() == ("reserve_slot[2]",)
    assert planner.has_placeholders


def test_nothing_is_listed_only_when_every_value_is_confirmed(tmp_path: Path) -> None:
    reserve = tuple(ReserveSlot(piece, is_placeholder=False) for piece in TEST_RESERVE)
    planner = planner_with(real_grasp(tmp_path), reserve)
    assert planner.placeholders() == ()
    assert not planner.has_placeholders
    assert planner.grasp is not None and not planner.grasp.is_placeholder


def test_mixed_input_keeps_each_slot_flag_in_zone_order() -> None:
    reserve = (ReserveSlot(Q, False), R, ReserveSlot(B, True), N)  # a bare piece type is a placeholder
    planner = planner_with(GraspProfiles.from_dict(TEST_GRASP, placeholder=False), reserve)
    assert planner.reserve_pieces == (Q, R, B, N)
    assert planner.placeholders() == ("reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]")


def test_the_flag_is_information_only_the_chain_is_the_same(tmp_path: Path) -> None:
    placeholder = make_planner()
    confirmed = planner_with(
        real_grasp(tmp_path), tuple(ReserveSlot(piece, is_placeholder=False) for piece in TEST_RESERVE)
    )
    board = chess.Board()
    assert placeholder.plan(board, E2E4).describe() == confirmed.plan(board, E2E4).describe()
    assert placeholder.has_placeholders and not confirmed.has_placeholders


def test_a_planner_without_a_reserve_zone_has_no_reserve_placeholders(tmp_path: Path) -> None:
    planner = MovePlanner(make_workspace(), make_geometry(), make_zones(reserve_slots=None), real_grasp(tmp_path))
    assert planner.reserve_slots == () and planner.placeholders() == ()


# --------------------------------------------------------------------------- #
# The console runner's demo values are placeholders, and says so
# --------------------------------------------------------------------------- #


def test_the_runner_demo_values_are_all_placeholders() -> None:
    grasp = GraspProfiles.from_dict(console_runner.PLACEHOLDER_GRASP, placeholder=True)
    reserve = tuple(ReserveSlot(piece, is_placeholder=True) for piece in console_runner.PLACEHOLDER_RESERVE_PIECES)
    planner = planner_with(grasp, reserve)
    assert planner.placeholders() == (
        "grasp_profiles", "reserve_slot[0]", "reserve_slot[1]", "reserve_slot[2]", "reserve_slot[3]",
    )


def test_the_command_line_lists_what_is_still_a_placeholder(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(builtins, "input", lambda _prompt="": "quit")
    assert console_runner.main(["--demo-placeholders", "--data-dir", str(tmp_path), "--new"]) == 0
    printed = capsys.readouterr().out
    assert "Still placeholders" in printed and "grasp_profiles" in printed and "reserve_slot[3]" in printed
