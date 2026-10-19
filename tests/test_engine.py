"""Engine: config, the five modes, FEN-only queries, restart on timeout (SPEC section 9).

No test here needs the Stockfish binary except the one marked integration, which is skipped
when the binary cannot be found.
"""

from __future__ import annotations

import asyncio
import json
import re
import shutil
from pathlib import Path
from typing import Any

import chess
import chess.engine
import pytest

from desktop_app.core.engine import (
    MODE_KEYS,
    Engine,
    EngineConfigError,
    EngineCrashedError,
    EngineError,
    EngineRestarted,
    EngineTimeoutError,
    EngineUnavailableError,
    StockfishBackend,
    load_engine_config,
)
from desktop_app.core.event_bus import EventBus
from phase2_support import E2E4, FakeBackends, engine_config

REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO_ROOT / "config" / "difficulty_levels.json"
AFTER_E4 = "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1"


def make_engine(script: list[object] | None = None, mode: str | None = None, bus: EventBus | None = None) -> tuple[Engine, FakeBackends]:
    backends = FakeBackends(script)
    return Engine(engine_config(), backend_factory=backends, mode=mode, bus=bus), backends


def config_data() -> dict[str, Any]:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def write_config(tmp_path: Path, data: dict[str, Any]) -> Path:
    path = tmp_path / "difficulty_levels.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


# --------------------------------------------------------------------------- #
# config/difficulty_levels.json
# --------------------------------------------------------------------------- #


def test_the_five_modes_have_english_internal_keys() -> None:
    assert MODE_KEYS == ("beginner", "amateur", "player", "master", "grandmaster")
    assert list(engine_config().modes) == list(MODE_KEYS)


def test_modes_match_the_spec_table() -> None:
    modes = engine_config().modes
    # SPEC section 9: Skill Level 0 + depth <= 2 at 0.2 s; Elo 1400 / 1800 / 2400; full strength at 3 s.
    assert modes["beginner"].options == {"Skill Level": 0}
    assert (modes["beginner"].think_time_s, modes["beginner"].max_depth) == (0.2, 2)
    for key, elo, seconds in (("amateur", 1400, 0.5), ("player", 1800, 1.0), ("master", 2400, 2.0)):
        assert modes[key].options == {"UCI_LimitStrength": True, "UCI_Elo": elo}, key
        assert modes[key].think_time_s == seconds and modes[key].max_depth is None, key
    assert modes["grandmaster"].options == {}
    assert modes["grandmaster"].think_time_s == 3.0


def test_config_file_is_ascii_english_only() -> None:
    assert re.fullmatch(r"[\x00-\x7f]*", CONFIG_PATH.read_text(encoding="utf-8")), "no Vietnamese outside strings_vi.py"


def test_engine_path_comes_from_the_config() -> None:
    assert engine_config().path == config_data()["engine"]["path"]


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda d: d["modes"].pop("master"), "modes must be exactly"),
        (lambda d: d["modes"].update(extra={}), "modes must be exactly"),
        (lambda d: d["modes"]["beginner"].update(think_time_s=0), "think_time_s"),
        (lambda d: d["modes"]["beginner"].update(think_time_s=True), "think_time_s"),
        (lambda d: d["modes"]["beginner"].update(max_depth=True), "max_depth"),
        (lambda d: d["modes"]["beginner"].update(max_depth=0), "max_depth"),
        (lambda d: d["modes"]["beginner"].update(options={"Skill Level": 1.5}), "options"),
        (lambda d: d.update(default_mode="expert"), "default_mode"),
        (lambda d: d["engine"].update(path=""), "engine.path"),
        (lambda d: d["engine"].update(answer_margin_s=-1), "answer_margin_s"),
        (lambda d: d["engine"].update(max_restarts=-1), "max_restarts"),
        (lambda d: d.pop("engine"), "missing key"),
    ],
)
def test_invalid_config_is_rejected(tmp_path: Path, mutate: Any, message: str) -> None:
    data = config_data()
    mutate(data)
    with pytest.raises(EngineConfigError, match=message):
        load_engine_config(write_config(tmp_path, data))


def test_missing_or_broken_config_file(tmp_path: Path) -> None:
    with pytest.raises(EngineConfigError):
        load_engine_config(tmp_path / "nope.json")
    broken = tmp_path / "broken.json"
    broken.write_text("{")
    with pytest.raises(EngineConfigError):
        load_engine_config(broken)


# --------------------------------------------------------------------------- #
# Modes and options
# --------------------------------------------------------------------------- #


def test_best_move_applies_the_mode_options_and_limits() -> None:
    engine, fake = make_engine(mode="amateur")
    engine.best_move(chess.STARTING_FEN)
    backend = fake.backends[0]
    assert backend.configured == [{"UCI_LimitStrength": True, "Skill Level": 20, "UCI_Elo": 1400}]
    assert (fake.plays[0]["time_s"], fake.plays[0]["depth"]) == (0.5, None)
    assert fake.plays[0]["margin"] == engine_config().answer_margin_s


def test_beginner_limits_depth_and_uses_skill_level() -> None:
    engine, fake = make_engine(mode="beginner")
    engine.best_move(chess.STARTING_FEN)
    assert fake.backends[0].configured == [{"UCI_LimitStrength": False, "Skill Level": 0}]
    assert (fake.plays[0]["time_s"], fake.plays[0]["depth"]) == (0.2, 2)


def test_every_mode_turns_strength_limiting_on_or_off_explicitly() -> None:
    """Options of the previous mode must never leak into the next one."""
    engine, fake = make_engine(mode="beginner")
    for key in ("amateur", "grandmaster", "beginner"):
        engine.best_move(chess.STARTING_FEN)
        engine.set_mode(key)
    engine.best_move(chess.STARTING_FEN)
    configured = fake.backends[0].configured
    assert configured[0] == {"UCI_LimitStrength": False, "Skill Level": 0}
    assert configured[1] == {"UCI_LimitStrength": True, "Skill Level": 20, "UCI_Elo": 1400}
    assert configured[2] == {"UCI_LimitStrength": False, "Skill Level": 20}  # full strength, skill reset
    assert configured[3] == {"UCI_LimitStrength": False, "Skill Level": 0}


def test_a_new_mode_takes_effect_from_the_next_engine_move() -> None:
    engine, fake = make_engine(mode="player")
    engine.best_move(chess.STARTING_FEN)
    engine.set_mode("master")
    assert engine.mode == "master"
    assert len(fake.backends[0].configured) == 1  # nothing is sent until the next move
    engine.best_move(chess.STARTING_FEN)
    assert fake.backends[0].configured[-1]["UCI_Elo"] == 2400
    assert fake.plays[-1]["time_s"] == 2.0


def test_options_are_not_resent_when_the_mode_did_not_change() -> None:
    engine, fake = make_engine(mode="player")
    engine.best_move(chess.STARTING_FEN)
    engine.best_move(chess.STARTING_FEN)
    assert len(fake.backends[0].configured) == 1


def test_unknown_mode_is_rejected() -> None:
    engine, _ = make_engine()
    with pytest.raises(EngineConfigError):
        engine.set_mode("expert")
    assert engine.mode == engine_config().default_mode


# --------------------------------------------------------------------------- #
# Always the FEN from GameState
# --------------------------------------------------------------------------- #


def test_the_engine_is_queried_with_the_fen_and_no_move_history() -> None:
    engine, fake = make_engine()
    move = engine.best_move(AFTER_E4)
    assert fake.plays[0]["fen"] == AFTER_E4
    assert fake.plays[0]["stack"] == 0  # a fresh board from the FEN, no move stack
    assert move in chess.Board(AFTER_E4).legal_moves


def test_an_illegal_answer_is_never_returned() -> None:
    engine, _ = make_engine(script=[E2E4])  # e2e4 is not legal for black
    with pytest.raises(EngineError, match="illegal"):
        engine.best_move(AFTER_E4)


def test_positions_without_a_legal_move_or_with_a_bad_fen_are_refused() -> None:
    engine, fake = make_engine()
    with pytest.raises(EngineError, match="no legal move"):
        engine.best_move("7k/5Q2/6K1/8/8/8/8/8 b - - 0 1")
    with pytest.raises(EngineError, match="invalid FEN"):
        engine.best_move("garbage")
    assert fake.created == 0  # the process is not even started


# --------------------------------------------------------------------------- #
# Restart and reload the FEN when the engine does not answer
# --------------------------------------------------------------------------- #


def test_a_timeout_restarts_the_engine_and_reloads_the_fen() -> None:
    bus = EventBus()
    restarts: list[EngineRestarted] = []
    bus.subscribe(EngineRestarted, restarts.append)
    engine, fake = make_engine(script=[EngineTimeoutError("no answer"), "ok"], mode="master", bus=bus)

    move = engine.best_move(AFTER_E4)

    assert move in chess.Board(AFTER_E4).legal_moves
    assert fake.created == 2
    assert fake.backends[0].quit_called  # the hung process was stopped
    assert [p["fen"] for p in fake.plays] == [AFTER_E4, AFTER_E4]  # the same FEN again
    assert fake.plays[0]["backend"] is not fake.plays[1]["backend"]
    assert fake.backends[1].configured == [{"UCI_LimitStrength": True, "Skill Level": 20, "UCI_Elo": 2400}]
    assert len(restarts) == 1 and restarts[0].attempt == 1 and "no answer" in restarts[0].reason


def test_a_crashed_engine_is_restarted_too() -> None:
    engine, fake = make_engine(script=[EngineCrashedError("died"), "ok"])
    engine.best_move(chess.STARTING_FEN)
    assert fake.created == 2


def test_the_engine_gives_up_after_the_configured_restarts() -> None:
    config = engine_config()
    engine, fake = make_engine(script=[EngineTimeoutError("t")] * (config.max_restarts + 1))
    with pytest.raises(EngineUnavailableError):
        engine.best_move(chess.STARTING_FEN)
    assert fake.created == config.max_restarts + 1 and len(fake.plays) == config.max_restarts + 1
    assert all(b.quit_called for b in fake.backends)


def test_the_engine_recovers_on_the_next_call_after_giving_up() -> None:
    engine, fake = make_engine(script=[EngineTimeoutError("t")] * (engine_config().max_restarts + 1))
    with pytest.raises(EngineUnavailableError):
        engine.best_move(chess.STARTING_FEN)
    assert engine.best_move(chess.STARTING_FEN) in chess.Board().legal_moves


def test_a_launch_failure_is_reported() -> None:
    def refuse() -> Any:
        raise EngineUnavailableError("cannot start the engine")

    engine = Engine(engine_config(), backend_factory=refuse)
    with pytest.raises(EngineUnavailableError):
        engine.best_move(chess.STARTING_FEN)


def test_close_stops_the_process_and_is_repeatable() -> None:
    engine, fake = make_engine()
    engine.best_move(chess.STARTING_FEN)
    engine.close()
    engine.close()
    assert fake.backends[0].quit_called


def test_context_manager_closes_the_engine() -> None:
    backends = FakeBackends()
    with Engine(engine_config(), backend_factory=backends) as engine:
        engine.best_move(chess.STARTING_FEN)
    assert backends.backends[0].quit_called


# --------------------------------------------------------------------------- #
# StockfishBackend error mapping (no binary: python-chess is patched)
# --------------------------------------------------------------------------- #


class _StubSimpleEngine:
    def __init__(self, error: BaseException | None = None, move: chess.Move | None = chess.Move.from_uci("e2e4")) -> None:
        self.error = error
        self.move = move
        self.timeout: float | None = None
        self.limit: chess.engine.Limit | None = None
        self.options: dict[str, Any] = {}
        self.quit_error: BaseException | None = None
        self.closed = False

    def play(self, board: chess.Board, limit: chess.engine.Limit) -> Any:
        self.limit = limit
        if self.error is not None:
            raise self.error
        return chess.engine.PlayResult(self.move, None)

    def configure(self, options: dict[str, Any]) -> None:
        if self.error is not None:
            raise self.error
        self.options = options

    def quit(self) -> None:
        if self.quit_error is not None:
            raise self.quit_error

    def close(self) -> None:
        self.closed = True


def backend_with(monkeypatch: pytest.MonkeyPatch, stub: _StubSimpleEngine) -> StockfishBackend:
    monkeypatch.setattr(chess.engine.SimpleEngine, "popen_uci", staticmethod(lambda path: stub))
    return StockfishBackend("stockfish")


def test_backend_passes_time_depth_and_margin(monkeypatch: pytest.MonkeyPatch) -> None:
    stub = _StubSimpleEngine()
    backend = backend_with(monkeypatch, stub)
    assert backend.play(chess.Board(), time_s=0.2, depth=2, answer_margin_s=5.0) == chess.Move.from_uci("e2e4")
    assert stub.timeout == 5.0 and stub.limit is not None and (stub.limit.time, stub.limit.depth) == (0.2, 2)


@pytest.mark.parametrize("error", [asyncio.TimeoutError(), TimeoutError()])
def test_backend_maps_timeouts(monkeypatch: pytest.MonkeyPatch, error: BaseException) -> None:
    backend = backend_with(monkeypatch, _StubSimpleEngine(error))
    with pytest.raises(EngineTimeoutError):
        backend.play(chess.Board(), time_s=1.0, depth=None, answer_margin_s=1.0)


def test_backend_maps_a_dead_process_and_protocol_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    for error in (chess.engine.EngineTerminatedError(), chess.engine.EngineError("bad")):
        backend = backend_with(monkeypatch, _StubSimpleEngine(error))
        with pytest.raises(EngineCrashedError):
            backend.play(chess.Board(), time_s=1.0, depth=None, answer_margin_s=1.0)
    backend = backend_with(monkeypatch, _StubSimpleEngine(move=None))
    with pytest.raises(EngineCrashedError):
        backend.play(chess.Board(), time_s=1.0, depth=None, answer_margin_s=1.0)


def test_backend_separates_rejected_options_from_a_dead_engine(monkeypatch: pytest.MonkeyPatch) -> None:
    rejected = backend_with(monkeypatch, _StubSimpleEngine(chess.engine.EngineError("unknown option")))
    with pytest.raises(EngineConfigError):
        rejected.configure({"Nope": 1})
    dead = backend_with(monkeypatch, _StubSimpleEngine(chess.engine.EngineTerminatedError()))
    with pytest.raises(EngineCrashedError):
        dead.configure({"Skill Level": 0})


def test_backend_quit_falls_back_to_close(monkeypatch: pytest.MonkeyPatch) -> None:
    stub = _StubSimpleEngine()
    stub.quit_error = RuntimeError("hung")
    backend_with(monkeypatch, stub).quit()
    assert stub.closed


def test_backend_reports_a_missing_binary() -> None:
    with pytest.raises(EngineUnavailableError, match="cannot start"):
        StockfishBackend("/nonexistent/stockfish-binary")


# --------------------------------------------------------------------------- #
# Integration with the real binary (skipped when Stockfish is not installed)
# --------------------------------------------------------------------------- #


def _stockfish_path() -> str | None:
    configured = engine_config().path
    return shutil.which(configured) or (configured if Path(configured).is_file() else None)


@pytest.mark.integration
@pytest.mark.skipif(_stockfish_path() is None, reason="the Stockfish binary configured in difficulty_levels.json is not installed")
def test_real_stockfish_answers_in_the_fast_modes_and_after_a_forced_restart() -> None:
    config = engine_config()
    path = _stockfish_path()
    assert path is not None
    fast = type(config)(
        path=path,
        answer_margin_s=config.answer_margin_s,
        max_restarts=config.max_restarts,
        default_options=config.default_options,
        default_mode="beginner",
        modes=config.modes,
    )
    with Engine(fast) as engine:
        for key in ("beginner", "amateur"):  # 0.2 s and 0.5 s: grandmaster's 3 s is not worth waiting for here
            engine.set_mode(key)
            assert engine.best_move(chess.STARTING_FEN) in chess.Board().legal_moves
        engine._backend.quit()  # type: ignore[union-attr]  # kill the process behind the engine's back
        assert engine.best_move(AFTER_E4) in chess.Board(AFTER_E4).legal_moves  # restarted, FEN reloaded
