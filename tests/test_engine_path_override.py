"""Stockfish path override: STOCKFISH_PATH, then config/local.yaml, then config/settings.yaml.

Nothing here needs the Stockfish binary. Every test passes its own ``environ`` and
``local_config_path`` unless it is specifically about the default wiring.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Optional

import chess
import pytest
import yaml

from desktop_app.core import engine as engine_module
from desktop_app.core.engine import Engine, EngineConfigError, load_engine_config
from desktop_app.utils import settings as settings_module
from desktop_app.utils.settings import DEFAULT_SETTINGS_PATH, STOCKFISH_PATH_ENV, PathSource
from phase2_support import FakeBackend, FakeBackends

REPO_ROOT = Path(__file__).resolve().parents[1]
FILE_PATH = yaml.safe_load(DEFAULT_SETTINGS_PATH.read_text(encoding="utf-8"))["engine"]["path"]

ENV_PATH = "/from/env/stockfish"
LOCAL_PATH = "/from/local/stockfish"


def write_local(tmp_path: Path, content: Any) -> Path:
    path = tmp_path / "local.yaml"
    path.write_text(content if isinstance(content, str) else yaml.safe_dump(content), encoding="utf-8")
    return path


def load(environ: Mapping[str, str], local: Optional[Path]) -> Any:
    return load_engine_config(environ=environ, local_config_path=local)


# --------------------------------------------------------------------------- #
# Precedence: environment > local file > settings.yaml
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    ("env", "local", "expected_path", "expected_source"),
    [
        pytest.param(ENV_PATH, LOCAL_PATH, ENV_PATH, PathSource.ENVIRONMENT, id="env beats local and file"),
        pytest.param(ENV_PATH, None, ENV_PATH, PathSource.ENVIRONMENT, id="env beats file"),
        pytest.param(None, LOCAL_PATH, LOCAL_PATH, PathSource.LOCAL_FILE, id="local beats file"),
        pytest.param(None, None, FILE_PATH, PathSource.SETTINGS_FILE, id="file when nothing overrides"),
        pytest.param("", LOCAL_PATH, LOCAL_PATH, PathSource.LOCAL_FILE, id="blank env is unset, local wins"),
        pytest.param("   ", None, FILE_PATH, PathSource.SETTINGS_FILE, id="whitespace env is unset, file wins"),
    ],
)
def test_precedence(
    tmp_path: Path, env: Optional[str], local: Optional[str], expected_path: str, expected_source: PathSource
) -> None:
    environ = {STOCKFISH_PATH_ENV: env} if env is not None else {}
    local_file = write_local(tmp_path, {"engine": {"path": local}}) if local is not None else tmp_path / "absent.yaml"
    config = load(environ, local_file)
    assert config.path == expected_path
    assert config.path_source is expected_source


def test_the_override_replaces_only_the_path_never_the_modes() -> None:
    plain = load_engine_config(environ={}, local_config_path=Path("absent.yaml"))
    overridden = load_engine_config(environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=Path("absent.yaml"))
    assert overridden.path == ENV_PATH and plain.path == FILE_PATH
    assert overridden.modes == plain.modes
    assert (overridden.default_mode, overridden.answer_margin_s, overridden.max_restarts) == (
        plain.default_mode,
        plain.answer_margin_s,
        plain.max_restarts,
    )


def test_values_are_stripped(tmp_path: Path) -> None:
    assert load({STOCKFISH_PATH_ENV: f"  {ENV_PATH}\n"}, None).path == ENV_PATH
    assert load({}, write_local(tmp_path, {"engine": {"path": f" {LOCAL_PATH} "}})).path == LOCAL_PATH


def test_the_override_applies_to_any_settings_file(tmp_path: Path) -> None:
    data = yaml.safe_load(DEFAULT_SETTINGS_PATH.read_text(encoding="utf-8"))
    data["engine"]["path"] = "some-other-engine"
    alternative = tmp_path / "alt.yaml"
    alternative.write_text(yaml.safe_dump(data), encoding="utf-8")

    assert load_engine_config(alternative, environ={}, local_config_path=tmp_path / "absent.yaml").path == "some-other-engine"
    assert load_engine_config(alternative, environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=tmp_path / "absent.yaml").path == ENV_PATH


# --------------------------------------------------------------------------- #
# The local file: optional, strict when present
# --------------------------------------------------------------------------- #


def test_a_missing_local_file_overrides_nothing(tmp_path: Path) -> None:
    assert load({}, tmp_path / "absent.yaml").path_source is PathSource.SETTINGS_FILE


def test_an_empty_local_file_or_object_overrides_nothing(tmp_path: Path) -> None:
    assert load({}, write_local(tmp_path, {})).path_source is PathSource.SETTINGS_FILE
    assert load({}, write_local(tmp_path, "")).path_source is PathSource.SETTINGS_FILE


@pytest.mark.parametrize(
    ("content", "message"),
    [
        pytest.param("engine: [", "not valid YAML", id="broken yaml"),
        pytest.param([1, 2], "expected a mapping", id="list"),
        pytest.param({"engine": {"pth": "typo"}}, "unknown key", id="typo in the key"),
        pytest.param({"engine": {"path": LOCAL_PATH}, "camera": "x"}, "unknown key", id="extra key"),
        pytest.param({"engine": {"path": 5}}, "engine.path", id="number"),
        pytest.param({"engine": {"path": None}}, "engine.path", id="null value"),
        pytest.param({"engine": {"path": ""}}, "engine.path", id="empty string"),
        pytest.param({"engine": {"path": "   "}}, "engine.path", id="blank string"),
        pytest.param({"assumed": []}, "assumed", id="the assumed list is not overridable"),
    ],
)
def test_a_broken_local_file_is_an_error_not_a_silent_fallback(tmp_path: Path, content: Any, message: str) -> None:
    with pytest.raises(EngineConfigError, match=message):
        load({}, write_local(tmp_path, content))


def test_an_unreadable_local_path_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(EngineConfigError, match="cannot read"):
        load({}, tmp_path)  # a directory


def test_a_broken_local_file_is_an_error_even_when_the_environment_decides_the_path(tmp_path: Path) -> None:
    """Unlike the old local.json (which held only the Stockfish path), local.yaml carries other keys (the serial port),
    so it is always read and validated; STOCKFISH_PATH only wins over the engine.path inside it."""
    broken = write_local(tmp_path, "engine: [")
    with pytest.raises(EngineConfigError, match="not valid YAML"):
        load({STOCKFISH_PATH_ENV: ENV_PATH}, broken)
    valid = write_local(tmp_path, {"engine": {"path": LOCAL_PATH}, "serial": {"port": "/dev/ttyUSB7"}})
    assert load({STOCKFISH_PATH_ENV: ENV_PATH}, valid).path == ENV_PATH


def test_the_settings_file_is_still_validated_when_it_is_overridden(tmp_path: Path) -> None:
    data = yaml.safe_load(DEFAULT_SETTINGS_PATH.read_text(encoding="utf-8"))
    data["engine"]["path"] = ""
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(EngineConfigError, match="engine.path"):
        load_engine_config(bad, environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=tmp_path / "absent.yaml")


# --------------------------------------------------------------------------- #
# Default wiring: the real environment and the repository's config/local.yaml
# --------------------------------------------------------------------------- #


def test_the_default_call_reads_the_process_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(STOCKFISH_PATH_ENV, ENV_PATH)
    config = load_engine_config()
    assert (config.path, config.path_source) == (ENV_PATH, PathSource.ENVIRONMENT)


def test_the_default_call_reads_the_default_local_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings_module, "DEFAULT_LOCAL_PATH", write_local(tmp_path, {"engine": {"path": LOCAL_PATH}}))
    config = load_engine_config()
    assert (config.path, config.path_source) == (LOCAL_PATH, PathSource.LOCAL_FILE)


def test_the_default_local_file_lives_next_to_the_other_config_files() -> None:
    # The conftest fixture replaces the constant for every test, so ask a fresh interpreter.
    code = (
        "import sys\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "from desktop_app.utils import settings\n"
        "print(settings.DEFAULT_LOCAL_PATH)\n"
        "print(settings.STOCKFISH_PATH_ENV)\n"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-c", code, str(REPO_ROOT)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    local_path, env_name = result.stdout.split()
    assert Path(local_path) == REPO_ROOT / "config" / "local.yaml"
    assert env_name == "STOCKFISH_PATH"


def test_the_test_suite_is_isolated_from_the_developers_machine() -> None:
    assert STOCKFISH_PATH_ENV not in os.environ
    assert not settings_module.DEFAULT_LOCAL_PATH.exists()
    assert load_engine_config().path_source is PathSource.SETTINGS_FILE


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
    config = load_engine_config(environ={STOCKFISH_PATH_ENV: ENV_PATH}, local_config_path=Path("absent.yaml"))
    with Engine(config) as engine:
        engine.best_move(chess.STARTING_FEN)
    assert RecordingBackend.launched == [ENV_PATH]


# --------------------------------------------------------------------------- #
# Repository hygiene
# --------------------------------------------------------------------------- #


def test_local_yaml_is_git_ignored() -> None:
    lines = [line.strip() for line in (REPO_ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()]
    assert "config/local.yaml" in lines


def test_the_repository_documents_local_yaml_with_an_example_file() -> None:
    """A developer may well have their own (git-ignored) config/local.yaml, so only the example is checked.

    The example must itself be a valid overlay: same keys as settings.yaml, correct types."""
    example = REPO_ROOT / "config" / "local.example.yaml"
    assert example.read_text(encoding="utf-8").isascii()
    settings = settings_module.load_settings(local_path=example, environ={})
    assert settings.engine.path and settings.serial.port
    assert settings.path_source is PathSource.LOCAL_FILE


# --------------------------------------------------------------------------- #
# The real-Stockfish test: its skipif and its body must agree
# --------------------------------------------------------------------------- #


def _fake_binary(tmp_path: Path) -> Path:
    """An executable file that is not a chess engine: enough for 'the binary exists'."""
    path = tmp_path / "fake-stockfish"
    path.write_text("not an engine\n", encoding="utf-8")
    path.chmod(0o755)
    return path


def test_locate_stockfish_follows_the_application_precedence(tmp_path: Path) -> None:
    from stockfish_support import locate_stockfish

    binary = _fake_binary(tmp_path)
    absent = tmp_path / "absent.yaml"
    assert locate_stockfish({STOCKFISH_PATH_ENV: str(binary)}, absent) == (str(binary), "")
    local = write_local(tmp_path, {"engine": {"path": str(binary)}})
    assert locate_stockfish({}, local) == (str(binary), "")
    # the environment wins over the local file: a missing env binary is NOT rescued by the local file
    path, reason = locate_stockfish({STOCKFISH_PATH_ENV: str(tmp_path / "gone")}, local)
    assert path is None and "gone" in reason and PathSource.ENVIRONMENT.value in reason
    path, reason = locate_stockfish({}, write_local(tmp_path, "[not a mapping]"))
    assert path is None and "invalid" in reason


def _run_real_stockfish_test(tmp_path: Path, environ_extra: dict[str, str]) -> subprocess.CompletedProcess[str]:
    env = {k: v for k, v in os.environ.items() if k != STOCKFISH_PATH_ENV}
    env.update(environ_extra)
    return subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "-rs", "tests/test_engine.py", "-k", "real_stockfish"],
        cwd=REPO_ROOT, env=env, capture_output=True, text=True, check=False, timeout=120,
    )


def test_a_configured_binary_makes_the_real_stockfish_test_run_not_skip_and_not_assert(tmp_path: Path) -> None:
    """Regression: the isolation fixture hid STOCKFISH_PATH from the body, so a found binary failed 'path is not None'."""
    result = _run_real_stockfish_test(tmp_path, {STOCKFISH_PATH_ENV: str(_fake_binary(tmp_path))})
    output = result.stdout + result.stderr
    assert "skipped" not in output.splitlines()[-1], output  # the skipif found the binary, so the test must run
    # it still fails (the file is not an engine), but on starting the engine, never on the path assertion
    assert "failed" in output.splitlines()[-1], output
    assert "AssertionError" not in output, output
    assert "EngineUnavailableError" in output, output


def test_without_a_binary_the_real_stockfish_test_is_skipped_with_the_reason(tmp_path: Path) -> None:
    result = _run_real_stockfish_test(tmp_path, {STOCKFISH_PATH_ENV: str(tmp_path / "missing-stockfish")})
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "1 skipped" in output and "missing-stockfish" in output and PathSource.ENVIRONMENT.value in output, output
