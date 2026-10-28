"""Move Planner: chains for normal moves, captures, en passant, castling, promotion (SPEC 6.4).

Every chain is also EXECUTED on the stub robot against a simulated physical board, and the
resulting squares must equal the position GameState.preview_move predicts. That checks the
meaning of the chain, not only its shape.
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Optional

import chess
import pytest

from desktop_app.console_runner import PhysicalWorld, StubRobot, suspect_squares
from desktop_app.core.game_state import GameState, MoveSource
from desktop_app.core.move_planner import (
    CaptureZoneFullError,
    GraspProfiles,
    GraspProfilesError,
    GripAction,
    GripStep,
    LocationKind,
    MoveChain,
    MovePlanError,
    MovePlanner,
    MoveStep,
    ReservePieceUnavailableError,
    TransferRole,
    ZDownStep,
    ZoneInventory,
    ZUpStep,
)
from desktop_app.kinematics.dimension import Workspace
from desktop_app.kinematics.five_bar import FiveBarKinematics, OutOfReachError
from phase2_support import CAPTURE_SLOTS, PARK_POSITION, RESERVE_SLOTS, TEST_GRASP, TEST_RESERVE, make_geometry, make_planner, make_workspace, make_zones

REPO_ROOT = Path(__file__).resolve().parents[1]
UCI = chess.Move.from_uci

# Positions for each kind of move.
NORMAL = ("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", "e2e4")
CAPTURE = ("4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1", "e4d5")
EN_PASSANT_WHITE = ("4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1", "e5d6")
EN_PASSANT_BLACK = ("4k3/8/8/8/3pP3/8/8/4K3 b - e3 0 1", "d4e3")
CASTLE_WHITE_KING = ("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1", "e1g1")
CASTLE_WHITE_QUEEN = ("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1", "e1c1")
CASTLE_BLACK_KING = ("r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1", "e8g8")
CASTLE_BLACK_QUEEN = ("r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1", "e8c8")
PROMOTION = ("4k3/P7/8/8/8/8/8/4K3 w - - 0 1", "a7a8q")
PROMOTION_CAPTURE = ("1n2k3/P7/8/8/8/8/8/4K3 w - - 0 1", "a7b8q")
PROMOTION_BLACK = ("4k3/8/8/8/8/8/p7/4K3 b - - 0 1", "a2a1q")


def plan(case: tuple[str, str], planner: Optional[MovePlanner] = None, inventory: Optional[ZoneInventory] = None) -> tuple[chess.Board, MoveChain]:
    fen, uci = case
    board = chess.Board(fen)
    return board, (planner or make_planner()).plan(board, UCI(uci), inventory)


def execute(board: chess.Board, chain: MoveChain, planner: Optional[MovePlanner] = None) -> dict[str, chess.Piece]:
    """Run the chain on a simulated physical board; the robot's spare pieces have the mover's colour."""
    planner = planner or make_planner()
    reserve = {
        f"reserve_slot[{i}]": chess.Piece(piece_type, board.turn) for i, piece_type in enumerate(planner.reserve_pieces)
    }
    world = PhysicalWorld(board, reserve)
    robot = StubRobot(world)
    robot.execute(chain)
    robot.park(chain)
    return world.squares()


def assert_chain_gives_the_expected_position(case: tuple[str, str], planner: Optional[MovePlanner] = None) -> MoveChain:
    board, chain = plan(case, planner)
    expected = GameState(fen=case[0]).preview_move(UCI(case[1]))
    assert suspect_squares(expected, execute(board, chain, planner)) == []
    return chain


# --------------------------------------------------------------------------- #
# Step sequence of one pick-and-place (SPEC 6.4)
# --------------------------------------------------------------------------- #


def test_normal_move_is_one_pick_and_place_in_the_spec_order() -> None:
    board, chain = plan(NORMAL)
    assert [t.role for t in chain.transfers] == [TransferRole.MOVING_PIECE]
    steps = chain.transfers[0].steps
    assert [type(s) for s in steps] == [MoveStep, ZDownStep, GripStep, ZUpStep, MoveStep, ZDownStep, GripStep, ZUpStep]
    assert [s.action for s in steps if isinstance(s, GripStep)] == [GripAction.CLOSE, GripAction.OPEN]
    assert [s.target.name for s in steps if isinstance(s, MoveStep)] == ["e2", "e4"]
    assert chain.san == "e4" and chain.move_uci == "e2e4"
    assert chain.capture_slots_used == () and chain.reserve_slot_used is None


def test_gripper_angles_come_from_the_grasp_profile_per_piece() -> None:
    _, chain = plan(NORMAL)  # a pawn
    close, open_ = [s for s in chain.steps if isinstance(s, GripStep)]
    assert close.angle_deg == TEST_GRASP["close_deg"]["P"] and open_.angle_deg == TEST_GRASP["open_deg"]
    _, rook = plan(("4k3/8/8/8/8/8/8/R3K3 w - - 0 1", "a1a5"))
    assert [s.angle_deg for s in rook.steps if isinstance(s, GripStep)][0] == TEST_GRASP["close_deg"]["R"]


def test_targets_are_the_square_centres_from_the_geometry() -> None:
    geometry = make_geometry()
    _, chain = plan(NORMAL)
    moves = [s for s in chain.steps if isinstance(s, MoveStep)]
    assert moves[0].target.xy == geometry.square_center_by_name("e2")
    assert moves[1].target.xy == geometry.square_center_by_name("e4")
    assert moves[1].target.kind is LocationKind.SQUARE


def test_trajectories_are_straight_lines_validated_end_to_end() -> None:
    _, chain = plan(NORMAL)
    first, second = [s for s in chain.steps if isinstance(s, MoveStep)]
    # leg 1 starts at the park position (the arm rests there between moves), leg 2 at e2
    assert tuple(first.trajectory.points_xy[0]) == pytest.approx(PARK_POSITION)
    assert tuple(second.trajectory.points_xy[0]) == pytest.approx(first.target.xy)
    assert tuple(second.trajectory.points_xy[-1]) == pytest.approx(second.target.xy)
    for step in (first, second, chain.park_leg):
        assert step.trajectory.min_margin_deg >= 15.0  # SPEC 6.2


def test_park_leg_returns_to_the_park_position() -> None:
    _, chain = plan(NORMAL)
    assert chain.park_leg.target.kind is LocationKind.PARK
    assert tuple(chain.park_leg.trajectory.points_xy[-1]) == pytest.approx(PARK_POSITION)
    assert tuple(chain.park_leg.trajectory.points_xy[0]) == pytest.approx(chain.transfers[-1].target.xy)


def test_describe_lists_every_step() -> None:
    _, chain = plan(CAPTURE)
    text = "\n".join(chain.describe())
    assert "capture_slot[0]" in text and "GRIP  close pawn" in text and "ZDOWN" in text and "park" in text


# --------------------------------------------------------------------------- #
# Special moves
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("case", [NORMAL, CAPTURE, EN_PASSANT_WHITE, EN_PASSANT_BLACK, CASTLE_WHITE_KING,
                                  CASTLE_WHITE_QUEEN, CASTLE_BLACK_KING, CASTLE_BLACK_QUEEN, PROMOTION,
                                  PROMOTION_CAPTURE, PROMOTION_BLACK])
def test_every_chain_leaves_the_board_exactly_as_the_move_predicts(case: tuple[str, str]) -> None:
    assert_chain_gives_the_expected_position(case)


def test_capture_removes_the_captured_piece_first() -> None:
    chain = assert_chain_gives_the_expected_position(CAPTURE)
    first, second = chain.transfers
    assert (first.role, first.piece_type) == (TransferRole.CAPTURED_PIECE, chess.PAWN)
    assert (first.source.name, first.target.name) == ("d5", "capture_slot[0]")
    assert first.target.kind is LocationKind.CAPTURE_SLOT
    assert first.target.xy == CAPTURE_SLOTS[0]
    assert (second.role, second.source.name, second.target.name) == (TransferRole.MOVING_PIECE, "e4", "d5")
    assert chain.capture_slots_used == (0,)


def test_en_passant_removes_the_pawn_from_its_own_square_not_the_target() -> None:
    chain = assert_chain_gives_the_expected_position(EN_PASSANT_WHITE)
    first, second = chain.transfers
    assert first.role is TransferRole.EN_PASSANT_PAWN
    assert (first.source.name, first.target.name) == ("d5", "capture_slot[0]")  # captured pawn is on d5
    assert (second.source.name, second.target.name) == ("e5", "d6")  # the capturing pawn lands on d6
    black = assert_chain_gives_the_expected_position(EN_PASSANT_BLACK)
    assert black.transfers[0].source.name == "e4" and black.transfers[1].target.name == "e3"


@pytest.mark.parametrize(
    "case, king, rook",
    [
        (CASTLE_WHITE_KING, ("e1", "g1"), ("h1", "f1")),
        (CASTLE_WHITE_QUEEN, ("e1", "c1"), ("a1", "d1")),
        (CASTLE_BLACK_KING, ("e8", "g8"), ("h8", "f8")),
        (CASTLE_BLACK_QUEEN, ("e8", "c8"), ("a8", "d8")),
    ],
)
def test_castling_moves_the_king_first_then_the_rook(case: tuple[str, str], king: tuple[str, str], rook: tuple[str, str]) -> None:
    chain = assert_chain_gives_the_expected_position(case)
    assert [t.role for t in chain.transfers] == [TransferRole.CASTLING_KING, TransferRole.CASTLING_ROOK]
    assert [t.piece_type for t in chain.transfers] == [chess.KING, chess.ROOK]
    assert [(t.source.name, t.target.name) for t in chain.transfers] == [king, rook]
    assert chain.capture_slots_used == ()


def test_promotion_takes_the_new_piece_from_the_reserve_zone() -> None:
    chain = assert_chain_gives_the_expected_position(PROMOTION)
    pawn, piece = chain.transfers
    assert (pawn.role, pawn.source.name, pawn.target.kind) == (TransferRole.PROMOTED_PAWN, "a7", LocationKind.CAPTURE_SLOT)
    assert (piece.role, piece.piece_type) == (TransferRole.PROMOTION_PIECE, chess.QUEEN)
    assert piece.source.kind is LocationKind.RESERVE_SLOT and piece.target.name == "a8"
    assert piece.source.xy == RESERVE_SLOTS[TEST_RESERVE.index(chess.QUEEN)]
    assert chain.reserve_slot_used == TEST_RESERVE.index(chess.QUEEN) and chain.capture_slots_used == (0,)


@pytest.mark.parametrize("letter, piece_type", [("q", chess.QUEEN), ("r", chess.ROOK), ("b", chess.BISHOP), ("n", chess.KNIGHT)])
def test_every_promotion_type_uses_the_matching_reserve_slot(letter: str, piece_type: chess.PieceType) -> None:
    case = (PROMOTION[0], f"a7a8{letter}")
    chain = assert_chain_gives_the_expected_position(case)
    assert chain.reserve_slot_used == TEST_RESERVE.index(piece_type)
    assert chain.transfers[-1].piece_type == piece_type


def test_promotion_with_capture_removes_the_victim_then_the_pawn_then_places_the_piece() -> None:
    chain = assert_chain_gives_the_expected_position(PROMOTION_CAPTURE)
    assert [t.role for t in chain.transfers] == [TransferRole.CAPTURED_PIECE, TransferRole.PROMOTED_PAWN, TransferRole.PROMOTION_PIECE]
    assert chain.transfers[0].piece_type == chess.KNIGHT and chain.transfers[0].source.name == "b8"
    assert chain.capture_slots_used == (0, 1)  # two pieces leave the board


# --------------------------------------------------------------------------- #
# Capture and reserve slots: next free slot, running out
# --------------------------------------------------------------------------- #


def test_captured_pieces_go_to_the_next_free_capture_slot() -> None:
    _, chain = plan(CAPTURE, inventory=ZoneInventory(capture_used=3))
    assert chain.capture_slots_used == (3,)
    assert chain.transfers[0].target.name == "capture_slot[3]"
    assert chain.transfers[0].target.xy == CAPTURE_SLOTS[3]


def test_consuming_a_chain_advances_the_inventory() -> None:
    inventory = ZoneInventory()
    _, first = plan(CAPTURE, inventory=inventory)
    inventory = inventory.consume(first)
    _, second = plan(CAPTURE, inventory=inventory)
    assert (first.capture_slots_used, second.capture_slots_used) == ((0,), (1,))
    assert inventory.consume(second).capture_used == 2


def test_planning_does_not_change_the_inventory_or_the_board() -> None:
    inventory = ZoneInventory(capture_used=2)
    board = chess.Board(CAPTURE[0])
    fen = board.fen()
    make_planner().plan(board, UCI(CAPTURE[1]), inventory)
    assert inventory == ZoneInventory(capture_used=2) and board.fen() == fen


def test_a_full_capture_zone_is_reported_and_no_chain_is_returned() -> None:
    planner = make_planner(capture_slots=2)
    plan(CAPTURE, planner, ZoneInventory(capture_used=1))  # the last free slot still works
    with pytest.raises(CaptureZoneFullError, match="full"):
        plan(CAPTURE, planner, ZoneInventory(capture_used=2))


def test_moves_that_remove_nothing_do_not_need_a_capture_slot() -> None:
    planner = make_planner(capture_slots=1)
    plan(NORMAL, planner, ZoneInventory(capture_used=1))
    plan(CASTLE_WHITE_KING, planner, ZoneInventory(capture_used=1))


def test_a_promotion_with_capture_needs_two_free_capture_slots() -> None:
    planner = make_planner(capture_slots=2)
    plan(PROMOTION_CAPTURE, planner, ZoneInventory(capture_used=0))
    with pytest.raises(CaptureZoneFullError):
        plan(PROMOTION_CAPTURE, planner, ZoneInventory(capture_used=1))  # only one slot left
    with pytest.raises(CaptureZoneFullError):
        plan(PROMOTION, planner, ZoneInventory(capture_used=2))  # the pawn also needs a slot


def test_running_out_of_reserve_pieces_is_reported() -> None:
    queen_slot = TEST_RESERVE.index(chess.QUEEN)
    used = ZoneInventory(reserve_used=frozenset({queen_slot}))
    with pytest.raises(ReservePieceUnavailableError, match="queen"):
        plan(PROMOTION, inventory=used)
    plan((PROMOTION[0], "a7a8r"), inventory=used)  # other piece types are still there


def test_a_second_spare_of_the_same_type_is_used_next() -> None:
    planner = make_planner(reserve_pieces=(chess.QUEEN, chess.QUEEN))
    _, first = plan(PROMOTION, planner)
    _, second = plan(PROMOTION, planner, ZoneInventory().consume(first))
    assert (first.reserve_slot_used, second.reserve_slot_used) == (0, 1)
    with pytest.raises(ReservePieceUnavailableError):
        plan(PROMOTION, planner, ZoneInventory().consume(first).consume(second))


def test_a_promotion_type_that_is_not_in_the_reserve_cannot_be_planned() -> None:
    planner = make_planner(reserve_pieces=(chess.QUEEN,))
    with pytest.raises(ReservePieceUnavailableError, match="knight"):
        plan((PROMOTION[0], "a7a8n"), planner)


def _engine_game(fen: str, *ucis: str) -> GameState:
    game = GameState(fen=fen)
    for text in ucis:
        game.set_pending_move(UCI(text))
        game.commit_move(UCI(text), MoveSource.ENGINE)
    return game


def test_inventory_is_rebuilt_from_engine_moves_only(tmp_path: Path) -> None:
    """The human handles the pieces he captures or promotes himself: they never use robot slots."""
    game = GameState.new_game(tmp_path, fen="4k3/P2p4/8/8/8/8/8/4K3 w - - 0 1")
    game.commit_move(UCI("a7a8q"), MoveSource.HUMAN)  # human promotes by hand
    game.commit_move(UCI("e8e7"), MoveSource.HUMAN)
    assert ZoneInventory.from_history(game.history, TEST_RESERVE) == ZoneInventory()


def test_inventory_counts_engine_captures_and_promotions() -> None:
    capture = _engine_game("4k3/8/8/3p4/4P3/8/8/4K3 b - - 0 1", "d5e4")
    assert ZoneInventory.from_history(capture.history, TEST_RESERVE) == ZoneInventory(capture_used=1)
    promotion = _engine_game(PROMOTION_BLACK[0], "a2a1q")  # the pawn leaves the board: one capture slot
    queen = frozenset({TEST_RESERVE.index(chess.QUEEN)})
    assert ZoneInventory.from_history(promotion.history, TEST_RESERVE) == ZoneInventory(capture_used=1, reserve_used=queen)


def test_inventory_after_a_restart_equals_the_inventory_before_it(tmp_path: Path) -> None:
    """A reloaded game must not drop a piece onto an occupied capture slot."""
    planner = make_planner()
    game = GameState.new_game(tmp_path, fen="4k3/8/8/3p4/4P3/8/8/4K3 b - - 0 1")
    inventory = ZoneInventory()
    for uci in ("d5e4",):
        move = UCI(uci)
        chain = planner.plan(game.board, move, inventory)
        game.set_pending_move(move)
        game.commit_move(move, MoveSource.ENGINE)
        inventory = inventory.consume(chain)
    reloaded = GameState.load(tmp_path)
    assert reloaded is not None
    assert ZoneInventory.from_history(reloaded.history, planner.reserve_pieces) == inventory == ZoneInventory(capture_used=1)


# --------------------------------------------------------------------------- #
# Workspace validation: an error BEFORE any chain exists
# --------------------------------------------------------------------------- #


class SpyWorkspace(Workspace):
    """Records every point that goes through check_point (the single entry point of Phase 1)."""

    def __init__(self, model: FiveBarKinematics, min_margin_deg: float) -> None:
        super().__init__(model, min_margin_deg)
        self.checked: list[tuple[float, float]] = []

    def check_point(self, x: float, y: float) -> float:
        self.checked.append((x, y))
        return super().check_point(x, y)


def test_every_location_and_trajectory_point_goes_through_check_point() -> None:
    spy = SpyWorkspace(FiveBarKinematics.from_config(), 15.0)
    board, chain = plan(CAPTURE, make_planner(workspace=spy))
    checked = {(round(x, 9), round(y, 9)) for x, y in spy.checked}
    for transfer in chain.transfers:
        for location in (transfer.source, transfer.target):
            assert (round(location.xy[0], 9), round(location.xy[1], 9)) in checked
    for step in [*chain.steps, chain.park_leg]:
        if isinstance(step, MoveStep):
            for x, y in step.trajectory.points_xy:
                assert (round(float(x), 9), round(float(y), 9)) in checked


def test_the_planner_source_never_calls_raw_ik() -> None:
    """The planner reaches kinematics only through Workspace.check_point / straight_line."""
    tree = ast.parse((REPO_ROOT / "desktop_app" / "core" / "move_planner.py").read_text(encoding="utf-8"))
    called = {node.func.attr for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert not called & {"ik", "fk", "margin_at_point", "angle_to_steps", "joints_to_steps"}, called
    assert {"check_point"} <= called


def _margin(square: str) -> float:
    workspace = make_workspace()
    return workspace.check_point(*make_geometry().square_center_by_name(square))


def test_a_point_below_the_margin_is_an_error_and_no_chain_is_returned() -> None:
    """A8 has mu about 26 degrees; with a 30 degree requirement it must be refused."""
    assert 26 <= _margin("a8") < 30
    strict = make_planner(workspace=make_workspace(min_margin_deg=30.0))
    board = chess.Board("r3k3/8/8/8/8/8/8/4K3 b - - 0 1")
    with pytest.raises(MovePlanError, match="a8"):
        strict.plan(board, UCI("a8a7"))
    # the same move is fine with the SPEC margin of 15 degrees
    assert make_planner().plan(board, UCI("a8a7")) is not None


def test_an_error_late_in_the_chain_still_returns_no_chain() -> None:
    """The first leg is fine; a later point (the drop on the bad square) fails. Nothing is returned."""
    strict = make_planner(workspace=make_workspace(min_margin_deg=30.0))
    board = chess.Board("4k3/8/8/8/8/8/8/R3K3 w - - 0 1")
    assert _margin("a1") >= 30  # the pick point is fine
    with pytest.raises(MovePlanError, match="a8"):
        strict.plan(board, UCI("a1a8"))


class InteriorFaultWorkspace(Workspace):
    """Passes every pick/drop point but fails points of a band that only a path crosses."""

    def check_point(self, x: float, y: float) -> float:
        if 150.0 <= y < 152.0:
            raise OutOfReachError(f"injected fault at ({x:g}, {y:g})")
        return super().check_point(x, y)


def test_a_bad_point_inside_a_leg_is_an_error_even_when_both_end_points_are_fine() -> None:
    workspace = InteriorFaultWorkspace(FiveBarKinematics.from_config(), 15.0)
    planner = make_planner(workspace=workspace)
    geometry = make_geometry()
    for square in ("e2", "e4"):  # the pick and the drop point are usable
        workspace.check_point(*geometry.square_center_by_name(square))
    with pytest.raises(MovePlanError, match="injected fault"):
        planner.plan(chess.Board(NORMAL[0]), UCI(NORMAL[1]))


def test_an_illegal_move_is_refused() -> None:
    with pytest.raises(MovePlanError, match="not a legal move"):
        make_planner().plan(chess.Board(), UCI("e2e5"))
    with pytest.raises(MovePlanError):
        make_planner().plan(chess.Board(), "e2e4")  # type: ignore[arg-type]


# --------------------------------------------------------------------------- #
# [TO CONFIRM] zones that are not provided yet
# --------------------------------------------------------------------------- #


def test_without_a_park_position_nothing_can_be_planned() -> None:
    with pytest.raises(MovePlanError, match="park_position"):
        plan(NORMAL, make_planner(park=False))


def test_an_undefined_capture_zone_blocks_captures_but_not_plain_moves() -> None:
    planner = make_planner(capture_slots=None)
    plan(NORMAL, planner)
    with pytest.raises(MovePlanError, match="capture_zone"):
        plan(CAPTURE, planner)


def test_an_undefined_reserve_zone_blocks_promotions() -> None:
    planner = make_planner(reserve_pieces=())
    plan(CAPTURE, planner)
    with pytest.raises(MovePlanError, match="reserve_zone"):
        plan(PROMOTION, planner)


def test_reserve_pieces_must_match_the_reserve_slots() -> None:
    with pytest.raises(MovePlanError, match="reserve_pieces"):
        MovePlanner(
            make_workspace(), make_geometry(), make_zones(), GraspProfiles.from_dict(TEST_GRASP),
            reserve_pieces=(chess.QUEEN,),
        )
    with pytest.raises(MovePlanError, match="promotion pieces"):
        make_planner(reserve_pieces=(chess.PAWN, chess.QUEEN, chess.ROOK, chess.BISHOP))


# --------------------------------------------------------------------------- #
# Grasp profiles (assumed values of settings.yaml: never invented in code)
# --------------------------------------------------------------------------- #


def test_grasp_profiles_come_from_settings_and_are_a_placeholder_while_assumed() -> None:
    from desktop_app.utils.settings import get_settings

    settings = get_settings()
    profiles = GraspProfiles.from_settings()
    assert profiles.open_deg == settings.grasp.open_deg
    assert profiles.close_angle(chess.KING) == settings.grasp.close_deg.K
    assert settings.assumed_under("grasp"), "the grasp angles are not confirmed on the real gripper yet"
    assert profiles.is_placeholder


def test_a_confirmed_grasp_is_not_a_placeholder(tmp_path: Path) -> None:
    import yaml

    from desktop_app.utils.settings import load_settings

    data = yaml.safe_load((REPO_ROOT / "config" / "settings.yaml").read_text(encoding="utf-8"))
    data["assumed"] = [k for k in data["assumed"] if not k.startswith("grasp.")]
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    assert not GraspProfiles.from_settings(load_settings(path, overlay=False)).is_placeholder


def test_the_repository_has_no_second_source_of_grasp_values() -> None:
    assert not (REPO_ROOT / "config" / "grasp_profiles.json").exists(), "gripper angles live in config/settings.yaml only"


@pytest.mark.parametrize(
    "data",
    [
        {},
        {"open_deg": 90},
        {"open_deg": 90, "close_deg": {"P": 1}},
        {"open_deg": -1, "close_deg": TEST_GRASP["close_deg"]},
        {"open_deg": True, "close_deg": TEST_GRASP["close_deg"]},
        {"open_deg": float("nan"), "close_deg": TEST_GRASP["close_deg"]},
        {"open_deg": 90, "close_deg": {**TEST_GRASP["close_deg"], "X": 5}},
        {"open_deg": 90, "close_deg": {**TEST_GRASP["close_deg"], "P": "high"}},
        {"open_deg": 90, "close_deg": TEST_GRASP["close_deg"], "extra": 1},
        [],
    ],
)
def test_invalid_grasp_profiles_are_rejected(data: object) -> None:
    with pytest.raises(GraspProfilesError):
        GraspProfiles.from_dict(data)


# --------------------------------------------------------------------------- #
# The stub robot really checks the chain
# --------------------------------------------------------------------------- #


def test_the_stub_robot_rejects_a_chain_with_a_wrong_piece() -> None:
    board, chain = plan(NORMAL)
    wrong = chess.Board("4k3/8/8/8/8/8/4Q3/4K3 w - - 0 1")  # a queen stands on e2, the chain grips a pawn
    world = PhysicalWorld(wrong, {})
    with pytest.raises(Exception, match="nothing to grip"):
        StubRobot(world).execute(chain)


def test_the_stub_robot_rejects_a_drop_on_an_occupied_square() -> None:
    _, chain = plan(CAPTURE)
    board = chess.Board(CAPTURE[0])
    world = PhysicalWorld(board, {})
    # Skip the capture's first transfer: the moving pawn then lands on the occupied d5.
    robot = StubRobot(world)
    with pytest.raises(Exception, match="occupied"):
        for step in chain.transfers[1].steps:
            robot._do(step)
