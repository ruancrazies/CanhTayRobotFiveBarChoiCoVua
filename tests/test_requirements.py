"""requirements.txt: pyserial is pinned to a range that was tested (phase 4a driver rules, decision 4)."""

from __future__ import annotations

import re
from importlib import metadata
from pathlib import Path

from packaging.specifiers import SpecifierSet
from packaging.version import Version

REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS = REPO_ROOT / "requirements.txt"
TESTED_PYSERIAL = Version("3.5")  # the version the suite was run with; update it together with the range


def _pyserial_specifier() -> SpecifierSet:
    lines = [l.split("#", 1)[0].strip() for l in REQUIREMENTS.read_text(encoding="utf-8").splitlines()]
    found = [l for l in lines if re.match(r"pyserial\b", l, re.I)]
    assert len(found) == 1, f"requirements.txt must name pyserial exactly once, got {found}"
    return SpecifierSet(found[0][len("pyserial"):])


def test_pyserial_has_a_lower_and_an_upper_bound() -> None:
    specifier = _pyserial_specifier()
    operators = {spec.operator for spec in specifier}
    assert ">=" in operators, "pyserial needs a lower bound"
    assert operators & {"<", "<=", "~=", "=="}, "pyserial needs an upper bound: a bare '>=' installs untested majors"


def test_the_tested_version_is_inside_the_range_and_a_new_major_is_not() -> None:
    specifier = _pyserial_specifier()
    assert TESTED_PYSERIAL in specifier
    assert Version("4.0") not in specifier


def test_the_installed_pyserial_is_the_tested_one_and_in_the_range() -> None:
    installed = Version(metadata.version("pyserial"))
    assert installed in _pyserial_specifier()
    assert installed == TESTED_PYSERIAL, f"the suite was tested with pyserial {TESTED_PYSERIAL}, this run has {installed}"
