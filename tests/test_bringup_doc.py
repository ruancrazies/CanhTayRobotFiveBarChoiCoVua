"""firmware/BRINGUP.md says what the driver rules look like on the board (phase 4a follow-up)."""

from __future__ import annotations

from pathlib import Path

BRINGUP = (Path(__file__).resolve().parents[1] / "firmware" / "BRINGUP.md").read_text(encoding="utf-8")


def test_the_4a_table_has_the_error_6_rows() -> None:
    assert "`DISABLE`, then `MOVE A=90 B=45 F=20` | `ok`, then `error:6`" in BRINGUP
    assert "(drivers disabled) `HOME`, `HOME AB`, `HOME Z AB` | `error:6` each" in BRINGUP
    assert "(drivers disabled) `HOME Z` | `error:1` | **not** `error:6`" in BRINGUP
    assert "`error:5` comes before `error:6`" in BRINGUP
    assert "MOVE A=90 B=45 F=20` (drivers enabled, no ESTOP) | `error:4`" in BRINGUP


def test_the_4b_section_covers_disable_during_a_pending_wait_and_a_running_home() -> None:
    section = BRINGUP[BRINGUP.index("## 5. What only becomes observable on hardware in stage 4b"):BRINGUP.index("## 6.")]
    assert "while `WAIT` is pending" in section and "**no `done` at all**" in section
    assert "`HOME` running, then `DISABLE`" in section and "**no `done`**" in section
    assert "`HOME Z` running, then `DISABLE`" in section and "then `done`" in section
    assert "`error:3`" in section and "`error:6` (not `error:4`)" in section


def test_the_s1_section_says_what_to_type_what_to_see_and_how_to_stop() -> None:
    section = BRINGUP[BRINGUP.index("## 8. Stage 4b-1, S1: joint motion"):]
    for needle in (
        "FastAccelStepper 0.33.14", "bench_move_without_homing", "Set it back to `false`", "**How to stop safely.**",
        "`POS A=90 B=45 SA=0 SB=0 NA=800 NB=400 Z=TOP`", "`error:3`", "`error:2`", "**no `done`**", "heartbeat",
        "joint1_dir_high_counts_up", "`INFO FW=4b1-s2 PROTO=1`",
    ):
        assert needle in section, needle
    assert "A=1 B=2 F=3" not in BRINGUP  # the old values are outside the joint limits now (error:2)


def test_the_firmware_version_in_the_doc_is_the_one_in_settings() -> None:
    from desktop_app.utils.settings import load_settings

    assert f"FW={load_settings(overlay=False).firmware.version} PROTO=" in BRINGUP


def test_the_s2_section_says_what_to_type_what_to_see_and_how_to_stop() -> None:
    section = BRINGUP[BRINGUP.index("## 9. Stage 4b-1, S2: the Z axis"):]
    for needle in (
        "`INFO FW=4b1-s2 PROTO=1`", "`!alarm:LIMIT_Z`", "`!alarm:Z_TIMEOUT`", "**How to stop safely.**", "`ZSTOP`", "`ZUP`", "`ZDOWN`",
        "D4", "electric brake", "full_travel_ms", "**no** `done`", "`DISABLE` never stops Z", "swap the two motor wires", "heartbeat",
    ):
        assert needle in section, needle
    assert "Z=MID" in section and "Z=TOP" in section and "Z=BOTTOM" in section
