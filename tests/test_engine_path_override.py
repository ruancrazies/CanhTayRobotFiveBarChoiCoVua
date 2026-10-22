"""Stockfish path override: STOCKFISH_PATH, then config/local.json, then difficulty_levels.json.

Nothing here needs the Stockfish binary. Every test passes its own ``environ`` and
``local_config_path`` unless it is specifically about the default wiring.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Optional

import chess
import pytest

from desktop_app.core import engine as engine_module
from desktop_app.core.engine import (
    DEFAULT_CONFIG_PATH,
    LOCAL_CONFIG_KEYS,
    STOCKFISH_PATH_ENV,
    Engine,
    EngineConfigError,
    PathSource,
    load_engine_config,
    resolve_engine_path,
)
from phase2_support import FakeBackend, FakeBackends

REPO_ROOT = Path(__file__).resolve().parents[1]
FILE_PATH = json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))["engine"]["path"]

ENV_PATH = "/from/env/stockfish"
LOCAL_PATH = "/from/local/stockfish"


def write_local(tmp_path: Path, content: Any) -> Path:
    path = tmp_path / "local.json"
    path.write_text(content if isinstance(content, str) else json.dumps(content), encoding="utf-8")
    return path


def load(environ: Mapping[str, str], local: Optional[Path]) -> Any:
    return load_engine_config(environ=environ, local_config_path=local)


# --------------------------------------------------------------------------- #
# Precedence: environment > local file > difficulty_levels.json
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    ("env", "local", "expected_path", "expected_source"),
    [
        pytest.param(ENV_PATH, LOCAL_PATH, ENV_PATH, PathSource.ENVIRONMENT, id="env beats local and file"),
        pytest.param(ENV_PATH, None, ENV_PATH, PathSource.ENVIRONMENT, id="env beats file"),
        pytest.param(None, LOCAL_PATH, LOCAL_PATH, PathSource.LOCAL_FILE, id="local beats file"),
        pytest.param(None, None, FILE_PATH, PathSource.DIFFICULTY_FILE, id="file when nothing overrides"),
        pytest.param("", LOCAL_PATH, LOCAL_PATH, PathSource.LOCAL_FILE, id="blank env is unset, local wins"),
        pytest.param("   ", None, FILE_PATH, PathSource.DIFFICULTY_FILE, id="whitespace env is unset, file wins"),
    ],
)
def test_precedence(
    tmp_path: Path, env: Optional[str], local: Optional[str], expected_path: str, expected_source: PathSource
) -> None:
    environ = {STOCKFISH_PATH_ENV: env} if env is not None else {}
    local_file = write_local(tmp_path, {"stockfish_path": local}) if local is not None else tmp_path / "absent.json"
    config = load(environ, local_file)
    assert config.path == expected_path
    assert config.path_source is expected_source


def test_the_override_replaces_only_the_path_never_the_modes() -> None:
    plain = load_engine_config(environ={}, local_config_path=Path("absent.json"))
    overridden = load_engine_config(environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=Path("absent.json"))
    assert overridden.path == ENV_PATH and plain.path == FILE_PATH
    assert overridden.modes == plain.modes
    assert (overridden.default_mode, overridden.answer_margin_s, overridden.max_restarts) == (
        plain.default_mode,
        plain.answer_margin_s,
        plain.max_restarts,
    )


def test_values_are_stripped(tmp_path: Path) -> None:
    assert load({STOCKFISH_PATH_ENV: f"  {ENV_PATH}\n"}, None).path == ENV_PATH
    assert load({}, write_local(tmp_path, {"stockfish_path": f" {LOCAL_PATH} "})).path == LOCAL_PATH


def test_the_override_applies_to_any_difficulty_file(tmp_path: Path) -> None:
    data = json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    data["engine"]["path"] = "some-other-engine"
    alternative = tmp_path / "alt.json"
    alternative.write_text(json.dumps(data), encoding="utf-8")

    assert load_engine_config(alternative, environ={}, local_config_path=tmp_path / "absent.json").path == "some-other-engine"
    assert load_engine_config(alternative, environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=tmp_path / "absent.json").path == ENV_PATH


def test_resolve_engine_path_returns_the_default_when_nothing_overrides(tmp_path: Path) -> None:
    assert resolve_engine_path("stockfish", environ={}, local_config_path=tmp_path / "absent.json") == (
        "stockfish",
        PathSource.DIFFICULTY_FILE,
    )


# --------------------------------------------------------------------------- #
# The local file: optional, strict when present
# --------------------------------------------------------------------------- #


def test_a_missing_local_file_overrides_nothing(tmp_path: Path) -> None:
    assert load({}, tmp_path / "absent.json").path_source is PathSource.DIFFICULTY_FILE


def test_an_empty_local_object_overrides_nothing(tmp_path: Path) -> None:
    assert load({}, write_local(tmp_path, {})).path_source is PathSource.DIFFICULTY_FILE


@pytest.mark.parametrize(
    ("content", "message"),
    [
        pytest.param("{", "not valid JSON", id="broken json"),
        pytest.param("", "not valid JSON", id="empty file"),
        pytest.param([1, 2], "top level must be an object", id="list"),
        pytest.param("null", "top level must be an object", id="null"),
        pytest.param({"stockfish_pth": "typo"}, "unknown key", id="typo in the key"),
        pytest.param({"stockfish_path": LOCAL_PATH, "camera": "x"}, "unknown key", id="extra key"),
        pytest.param({"stockfish_path": 5}, "non-empty string", id="number"),
        pytest.param({"stockfish_path": None}, "non-empty string", id="null value"),
        pytest.param({"stockfish_path": ""}, "non-empty string", id="empty string"),
        pytest.param({"stockfish_path": "   "}, "non-empty string", id="blank string"),
    ],
)
def test_a_broken_local_file_is_an_error_not_a_silent_fallback(tmp_path: Path, content: Any, message: str) -> None:
    with pytest.raises(EngineConfigError, match=message):
        load({}, write_local(tmp_path, content))


def test_an_unreadable_local_path_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(EngineConfigError, match="cannot read"):
        load({}, tmp_path)  # a directory


def test_the_environment_decides_without_reading_the_local_file(tmp_path: Path) -> None:
    """The local file is consulted only when the environment variable does not decide."""
    broken = write_local(tmp_path, "{")
    assert load({STOCKFISH_PATH_ENV: ENV_PATH}, broken).path == ENV_PATH
    with pytest.raises(EngineConfigError):
        load({}, broken)


def test_the_difficulty_file_is_still_validated_when_it_is_overridden(tmp_path: Path) -> None:
    data = json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    data["engine"]["path"] = ""
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(EngineConfigError, match="engine.path"):
        load_engine_config(bad, environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=tmp_path / "absent.json")


# --------------------------------------------------------------------------- #
# Default wiring: the real environment and the repository's config/local.json
# --------------------------------------------------------------------------- #


def test_the_default_call_reads_the_process_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(STOCKFISH_PATH_ENV, ENV_PATH)
    config = load_engine_config()
    assert (config.path, config.path_source) == (ENV_PATH, PathSource.ENVIRONMENT)


def test_the_default_call_reads_the_default_local_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(engine_module, "DEFAULT_LOCAL_CONFIG_PATH", write_local(tmp_path, {"stockfish_path": LOCAL_PATH}))
    config = load_engine_config()
    assert (config.path, config.path_source) == (LOCAL_PATH, PathSource.LOCAL_FILE)


def test_the_default_local_file_lives_next_to_the_other_config_files() -> None:
    # The conftest fixture replaces the constant for every test, so ask a fresh interpreter.
    code = (
        "import sys\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "from desktop_app.core import engine\n"
        "print(engine.DEFAULT_LOCAL_CONFIG_PATH)\n"
        "print(engine.STOCKFISH_PATH_ENV)\n"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-c", code, str(REPO_ROOT)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    local_path, env_name = result.stdout.split()
    assert Path(local_path) == REPO_ROOT / "config" / "local.json"
    assert env_name == "STOCKFISH_PATH"


def test_the_test_suite_is_isolated_from_the_developers_machine() -> None:
    assert STOCKFISH_PATH_ENV not in os.environ
    assert not engine_module.DEFAULT_LOCAL_CONFIG_PATH.exists()
    assert load_engine_config().path_source is PathSource.DIFFICULTY_FILE


# --------------------------------------------------------------------------- #
# The engine really launches the overridden path
# --------------------------------------------------------------------------- #


class RecordingBackend(FakeBackend):
    """Remembers the path it was started with."""

    launched: list[str] = []

    def __init__(self, path: str) -> None:
        super().__init__(FakeBackends())
        self.launched.append(path)


def test_the_engine_starts_the_overridden_binary(monkeypatch: pytest.MonkeyPatch) -> None:
    RecordingBackend.launched = []
    monkeypatch.setattr(engine_module, "StockfishBackend", RecordingBackend)
    config = load_engine_config(environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=Path("absent.json"))
    with Engine(config) as engine:
        engine.best_move(chess.STARTING_FEN)
    assert RecordingBackend.launched == [ENV_PATH]


# --------------------------------------------------------------------------- #
# Repository hygiene
# --------------------------------------------------------------------------- #


def test_local_json_is_git_ignored() -> None:
    lines = [line.strip() for line in (REPO_ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()]
    assert "config/local.json" in lines


def test_the_repository_documents_local_json_with_an_example_file() -> None:
    # A developer may well have their own (git-ignored) config/local.json, so only the example is checked.
    example = REPO_ROOT / "config" / "local.example.json"
    text = example.read_text(encoding="utf-8")
    assert text.isascii()
    data = json.loads(text)
    assert set(data) <= LOCAL_CONFIG_KEYS
    assert isinstance(data["stockfish_path"], str) and data["stockfish_path"].strip()
