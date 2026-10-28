"""docs/SPEC.md describes config/settings.yaml (rule 7, sections 3, 4 and 10.3): the text and the file must agree."""

from __future__ import annotations

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC = (REPO_ROOT / "docs" / "SPEC.md").read_text(encoding="utf-8")
SETTINGS = yaml.safe_load((REPO_ROOT / "config" / "settings.yaml").read_text(encoding="utf-8"))


def _section(title: str) -> str:
    match = re.search(rf"^#+ {re.escape(title)}.*?$(.*?)(?=^#{{2,3}} )", SPEC, re.S | re.M)
    assert match, title
    return match.group(1)


def test_rule_7_names_the_single_file_and_the_generator() -> None:
    rule = next(line for line in SPEC.splitlines() if line.startswith("7. "))
    for needle in ("config/settings.yaml", "config/local.yaml", "STOCKFISH_PATH", "python -m tools.gen_all", "ALLOWED_LITERALS"):
        assert needle in rule, needle


def test_the_spec_no_longer_names_the_removed_sources() -> None:
    for gone in ("hardware.yaml", "difficulty_levels.json", "grasp_profiles.json", "local.json"):
        assert gone not in SPEC, f"docs/SPEC.md still mentions {gone}"


def test_section_3_lists_the_new_files() -> None:
    layout = _section("3. Repository layout")
    for needle in ("settings.yaml", "local.yaml", "local.example.yaml", "tools/", "CONFIG_REFERENCE.md", "GENERATED"):
        assert needle in layout, needle


def test_section_4_describes_the_file_and_names_every_section_of_it() -> None:
    text = _section("4. Hardware constants and settings")
    for section in SETTINGS:
        if section == "assumed":
            continue
        assert f"`{section}`" in text, f"section 4 does not mention the settings section {section!r}"
    for needle in ("`assumed:`", "Every assumed key must exist in settings.yaml", "python -m tools.gen_all", "monitor_speed"):
        assert needle in text, needle


def test_section_10_3_describes_the_generated_headers() -> None:
    text = _section("10.3 Configuration of the firmware")
    for needle in (
        "GENERATED",
        "do not edit",
        "python -m tools.gen_firmware_config",
        "python -m tools.gen_all",
        "Every assumed key must exist in settings.yaml",
        "run_host_tests.sh",
        "pio run -e esp32dev",
    ):
        assert needle in text, needle


def test_the_values_in_the_section_4_tables_are_the_values_of_the_file() -> None:
    geometry, drive = SETTINGS["geometry"], SETTINGS["drive"]
    expected = {
        "| l | Distance between the two motor axes | 100 mm |": geometry["motor_axis_distance_mm"] == 100,
        "| m | Driving link length | 200 mm |": geometry["driving_link_mm"] == 200,
        "| n | Driven link length | 250 mm |": geometry["driven_link_mm"] == 250,
        "| k | Motor axes to near edge of board (border included) | 150 mm |": geometry["motor_to_board_edge_mm"] == 150,
        "| s | Square size | 31 mm |": geometry["square_size_mm"] == 31,
        "| border | Board border | 26 mm |": geometry["board_border_mm"] == 26,
        "| h | Z travel | 70 mm |": geometry["z_travel_mm"] == 70,
        "| f | TB6600 microstepping | 16 (SW1 OFF, SW2 OFF, SW3 ON) |": drive["microstepping"] == 16,
    }
    for row, agrees in expected.items():
        assert row in SPEC, row
        assert agrees, f"SPEC says {row!r} but settings.yaml differs"
