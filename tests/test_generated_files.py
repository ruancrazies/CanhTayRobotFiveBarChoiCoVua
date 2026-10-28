"""The files generated from config/settings.yaml must be current (SPEC rule 7 and section 10).

After editing config/settings.yaml run, from the repository root:

    python -m tools.gen_all

and commit the regenerated firmware/include/pins.h, firmware/include/config.h and docs/CONFIG_REFERENCE.md.
These tests regenerate each file in memory and fail when the committed file differs.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

from tools import gen_all, gen_config_reference, gen_firmware_config
from tools.gen_config_reference import REFERENCE_PATH, render_reference
from tools.gen_firmware_config import CONFIG_H, PINS_H, SETTINGS_PATH, render_config_h, render_pins_h

REGENERATE = "python -m tools.gen_all"


def _first_difference(committed: str, generated: str) -> str:
    for number, (a, b) in enumerate(zip(committed.splitlines(), generated.splitlines()), start=1):
        if a != b:
            return f"line {number}: committed {a!r} but settings.yaml generates {b!r}"
    return f"line counts differ: committed {len(committed.splitlines())}, generated {len(generated.splitlines())}"


def _assert_current(path: Path, generated: str) -> None:
    committed = path.read_text(encoding="utf-8")
    assert committed == generated, (
        f"{path.name} is stale "
        f"({_first_difference(committed, generated)}). config/settings.yaml is the source: run `{REGENERATE}` and commit."
    )


def test_pins_h_is_current() -> None:
    _assert_current(PINS_H, render_pins_h())


def test_config_h_is_current() -> None:
    _assert_current(CONFIG_H, render_config_h())


def test_config_reference_is_current() -> None:
    _assert_current(REFERENCE_PATH, render_reference())


@pytest.mark.parametrize("path", [PINS_H, CONFIG_H, REFERENCE_PATH], ids=lambda p: p.name)
def test_a_generated_file_says_generated_do_not_edit_on_its_first_line(path: Path) -> None:
    first = path.read_text(encoding="utf-8").splitlines()[0]
    assert "GENERATED" in first and "do not edit" in first and "settings.yaml" in first


@pytest.mark.parametrize("path", [PINS_H, CONFIG_H, REFERENCE_PATH], ids=lambda p: p.name)
def test_generated_files_are_ascii_with_unix_line_endings(path: Path) -> None:
    data = path.read_bytes()
    assert data.isascii() and b"\r" not in data and data.endswith(b"\n")


# --------------------------------------------------------------------------- #
# The drift tests do detect a change of settings.yaml
# --------------------------------------------------------------------------- #


def _edited_settings(tmp_path: Path, mutate) -> Path:  # type: ignore[no-untyped-def]
    data = yaml.safe_load(SETTINGS_PATH.read_text(encoding="utf-8"))
    mutate(data)
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def test_editing_a_pin_changes_pins_h_so_the_drift_test_would_fail(tmp_path: Path) -> None:
    edited = _edited_settings(tmp_path, lambda d: d["pins"].update(servo_pwm=23))
    assert render_pins_h(edited) != PINS_H.read_text(encoding="utf-8")
    assert "kServoPwm = 23;" in render_pins_h(edited)


def test_editing_the_baud_changes_config_h_so_the_drift_test_would_fail(tmp_path: Path) -> None:
    edited = _edited_settings(tmp_path, lambda d: d["serial"].update(baud=230400))
    assert render_config_h(edited) != CONFIG_H.read_text(encoding="utf-8")
    assert "kBaudRate = 230400;" in render_config_h(edited)


def test_editing_a_value_changes_the_reference_so_the_drift_test_would_fail(tmp_path: Path) -> None:
    edited = _edited_settings(tmp_path, lambda d: d["geometry"].update(square_size_mm=32.0))
    assert render_reference(edited) != REFERENCE_PATH.read_text(encoding="utf-8")


def test_confirming_a_value_changes_the_reference_and_the_header_marker(tmp_path: Path) -> None:
    edited = _edited_settings(tmp_path, lambda d: d["assumed"].remove("serial.baud"))
    assert render_reference(edited) != REFERENCE_PATH.read_text(encoding="utf-8")
    header = render_config_h(edited)
    assert header != CONFIG_H.read_text(encoding="utf-8")
    window = header[: header.index("kBaudRate =")].splitlines()[-3:]
    assert not any("ASSUMED" in line for line in window)


def test_an_invalid_settings_file_generates_nothing(tmp_path: Path) -> None:
    from desktop_app.utils.settings import ConfigError

    edited = _edited_settings(tmp_path, lambda d: d["pins"].update(servo_pwm=13))  # duplicate GPIO
    with pytest.raises(ConfigError):
        render_pins_h(edited)


def test_the_headers_do_not_depend_on_the_machine(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """local.yaml and STOCKFISH_PATH never reach a generated file."""
    from desktop_app.utils import settings as module

    local = tmp_path / "local.yaml"
    local.write_text("serial:\n  port: COM3\nengine:\n  path: /elsewhere/stockfish\n", encoding="utf-8")
    monkeypatch.setattr(module, "DEFAULT_LOCAL_PATH", local)
    monkeypatch.setenv(module.STOCKFISH_PATH_ENV, "/env/stockfish")
    assert render_pins_h() == PINS_H.read_text(encoding="utf-8")
    assert render_config_h() == CONFIG_H.read_text(encoding="utf-8")
    assert render_reference() == REFERENCE_PATH.read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# The command line
# --------------------------------------------------------------------------- #


def test_check_succeeds_when_everything_is_current(capsys: pytest.CaptureFixture[str]) -> None:
    assert gen_all.main(["--check"]) == 0
    assert gen_firmware_config.main(["--check"]) == 0
    assert gen_config_reference.main(["--check"]) == 0
    assert capsys.readouterr().err == ""


def test_check_fails_and_names_the_command_when_a_file_is_stale(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    stale = tmp_path / "pins.h"
    stale.write_text("// old\n", encoding="utf-8")
    monkeypatch.setattr(gen_firmware_config, "PINS_H", stale)
    monkeypatch.setattr(gen_firmware_config, "REPO_ROOT", tmp_path)
    assert gen_firmware_config.main(["--check"]) == 1
    assert REGENERATE in capsys.readouterr().err
    assert stale.read_text(encoding="utf-8") == "// old\n", "--check must not write"


def test_the_generator_writes_and_a_second_run_changes_nothing(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    pins, config = tmp_path / "pins.h", tmp_path / "config.h"
    monkeypatch.setattr(gen_firmware_config, "PINS_H", pins)
    monkeypatch.setattr(gen_firmware_config, "CONFIG_H", config)
    monkeypatch.setattr(gen_firmware_config, "REPO_ROOT", tmp_path)
    assert gen_firmware_config.main([]) == 0
    assert pins.read_text(encoding="utf-8") == PINS_H.read_text(encoding="utf-8")
    assert config.read_text(encoding="utf-8") == CONFIG_H.read_text(encoding="utf-8")
    pins.write_text("// edited by hand\n", encoding="utf-8")
    assert gen_firmware_config.main(["--check"]) == 1
    assert gen_firmware_config.main([]) == 0
    assert pins.read_text(encoding="utf-8") == PINS_H.read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# Content of the reference
# --------------------------------------------------------------------------- #


def test_the_reference_has_a_row_for_every_value_with_its_unit_and_assumed_flag() -> None:
    from tools.settings_doc import leaf_paths, load_documentation, unit_of

    raw, _comments = load_documentation()
    text = REFERENCE_PATH.read_text(encoding="utf-8")
    rows = {m.group(1): m.group(0) for m in re.finditer(r"^\| `([a-z0-9_.A-Z]+)` \|.*$", text, re.M)}
    assert set(rows) == set(leaf_paths(raw))
    assumed = set(raw["assumed"])
    for key, row in rows.items():
        cells = [c.strip() for c in row.strip("|").split("|")]
        assert cells[2] == unit_of(key), key
        assert (cells[3] == "yes") == (key in assumed), key
    assert text.count("\n- `") == len(assumed), "the assumed list at the end must name every assumed key"


def test_each_header_constant_is_documented_with_its_settings_key() -> None:
    text = PINS_H.read_text(encoding="utf-8") + CONFIG_H.read_text(encoding="utf-8")
    constants = gen_firmware_config.firmware_constants()
    for key, name in constants.items():
        block = re.search(rf"((?://[^\n]*\n)+)constexpr \w+ {name}\b", text)
        assert block, name
        assert f"// {key}" in block.group(1), f"{name} is not documented with its key {key}"
