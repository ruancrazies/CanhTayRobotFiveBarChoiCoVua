"""Structure rules of config/settings.yaml itself (SPEC section 4).

* every value has a comment saying what it is;
* every numeric value names its unit in the key (_ms, _deg_s, _deg_s2, _mm, _hz, ...) or is listed as dimensionless;
* every key of the top-level ``assumed`` list exists in the file (the rule "every assumed key must exist in settings.yaml");
* ASCII English only, no tabs, no duplicate keys.
"""

from __future__ import annotations

import re
from typing import Any

import pytest
import yaml

from desktop_app.utils.settings import DEFAULT_SETTINGS_PATH
from tools.settings_doc import DIMENSIONLESS, UNIT_SUFFIXES, leaf_paths, read_comments, unit_of, value_at

TEXT = DEFAULT_SETTINGS_PATH.read_text(encoding="utf-8")
RAW: dict[str, Any] = yaml.safe_load(TEXT)
LEAVES = leaf_paths(RAW)
COMMENTS = read_comments(TEXT)


def test_the_file_is_ascii_english_without_tabs() -> None:
    assert TEXT.isascii() and "\t" not in TEXT and "\r" not in TEXT


def test_there_are_no_duplicate_keys() -> None:
    class Strict(yaml.SafeLoader):
        pass

    def construct(loader: yaml.SafeLoader, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
        seen: set[Any] = set()
        for key_node, _ in node.value:
            key = loader.construct_object(key_node, deep=deep)
            assert key not in seen, f"duplicate key {key!r} (line {key_node.start_mark.line + 1})"
            seen.add(key)
        return loader.construct_mapping(node, deep)

    Strict.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct)
    yaml.load(TEXT, Loader=Strict)  # noqa: S506 - a SafeLoader subclass


def test_assumed_comes_first_and_is_a_list_of_dotted_strings() -> None:
    assert next(iter(RAW)) == "assumed"
    assert isinstance(RAW["assumed"], list) and RAW["assumed"]
    assert all(isinstance(key, str) and re.fullmatch(r"[a-z0-9_]+(\.[A-Za-z0-9_]+)+", key) for key in RAW["assumed"])
    assert len(set(RAW["assumed"])) == len(RAW["assumed"])


@pytest.mark.parametrize("key", RAW["assumed"])
def test_every_assumed_key_exists_in_settings_yaml(key: str) -> None:
    """Replaces the old rule "an ASSUMED constant must be listed": the list may only name keys that exist."""
    assert key in LEAVES, f"`assumed` names {key}, which is not a value of settings.yaml"
    value_at(RAW, key)


def test_every_section_value_is_either_a_leaf_or_a_listed_mapping() -> None:
    assert LEAVES and all("." in leaf for leaf in LEAVES)


def test_the_comment_reader_sees_exactly_the_values_of_the_file() -> None:
    assert set(COMMENTS) - {"assumed"} == set(LEAVES), (
        "the comment reader and the YAML parser disagree about the values: "
        f"{sorted(set(COMMENTS) ^ set(LEAVES))[:10]}"
    )


@pytest.mark.parametrize("key", LEAVES)
def test_every_value_has_a_comment(key: str) -> None:
    comment = COMMENTS.get(key, "").strip()
    assert len(comment) >= 8, f"{key} has no comment saying what it is (put it on the lines directly above the value)"


# Values that are null until set and are text, not numbers, so they have no unit.
NULL_TEXT_VALUES = frozenset({"serial.port"})


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _numbers_inside(value: Any) -> bool:
    if isinstance(value, (list, tuple)):
        return any(_is_number(v) for v in value)
    return _is_number(value)


@pytest.mark.parametrize("key", LEAVES)
def test_every_number_names_its_unit_or_is_listed_as_dimensionless(key: str) -> None:
    value = value_at(RAW, key)
    unit = unit_of(key)
    if key in DIMENSIONLESS:
        assert not unit, f"{key} is listed as dimensionless but its name carries the unit {unit}: remove it from DIMENSIONLESS"
    elif key in NULL_TEXT_VALUES:
        assert value is None or isinstance(value, str), f"{key} is a text value"
    elif _numbers_inside(value) or value is None:
        assert unit, f"{key} holds a number but its name has no unit suffix ({', '.join(s for s, _ in UNIT_SUFFIXES)}); rename it or add it to DIMENSIONLESS"


def test_dimensionless_lists_only_real_keys() -> None:
    assert sorted(DIMENSIONLESS - set(LEAVES)) == [], "remove keys that no longer exist from DIMENSIONLESS"


def test_a_unit_suffix_matches_its_comment_vocabulary() -> None:
    assert unit_of("motion.max_accel_deg_s2") == "deg/s^2"
    assert unit_of("motion.max_speed_deg_s") == "deg/s"
    assert unit_of("timeouts.heartbeat_timeout_ms") == "ms"
    assert unit_of("geometry.square_size_mm") == "mm"
    assert unit_of("electrical.z_axis.pwm_frequency_hz") == "Hz"
    assert unit_of("grasp.close_deg.P") == "deg"  # the unit is in the key above
    assert unit_of("serial.baud") == ""


def test_the_requested_unit_suffixes_exist() -> None:
    suffixes = {s for s, _ in UNIT_SUFFIXES}
    assert {"_ms", "_deg_s", "_deg_s2", "_mm", "_hz"} <= suffixes


def test_the_old_per_value_status_keys_are_gone() -> None:
    assert "to_confirm" not in RAW, "`assumed` replaced `to_confirm`"
