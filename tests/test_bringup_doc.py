"""firmware/BRINGUP.md says what the driver rules look like on the board (phase 4a follow-up)."""

from __future__ import annotations

from pathlib import Path

BRINGUP = (Path(__file__).resolve().parents[1] / "firmware" / "BRINGUP.md").read_text(encoding="utf-8")


def test_the_4a_table_has_the_error_6_rows() -> None:
    assert "`DISABLE`, then `MOVE A=1 B=2 F=3` | `ok`, then `error:6`" in BRINGUP
    assert "(drivers disabled) `HOME`, `HOME AB`, `HOME Z AB` | `error:6` each" in BRINGUP
    assert "(drivers disabled) `HOME Z` | `error:1` | **not** `error:6`" in BRINGUP
    assert "`error:5` comes before `error:6`" in BRINGUP
    assert "MOVE A=1 B=2 F=3` (drivers enabled, no ESTOP) | `error:4`" in BRINGUP


def test_the_4b_section_covers_disable_during_a_pending_wait_and_a_running_home() -> None:
    section = BRINGUP[BRINGUP.index("## 5. What only becomes observable on hardware in stage 4b"):BRINGUP.index("## 6.")]
    assert "while `WAIT` is pending" in section and "**no `done` at all**" in section
    assert "`HOME` running, then `DISABLE`" in section and "**no `done`**" in section
    assert "`HOME Z` running, then `DISABLE`" in section and "then `done`" in section
    assert "`error:3`" in section and "`error:6` (not `error:4`)" in section
