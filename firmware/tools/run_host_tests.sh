#!/usr/bin/env bash
# Host-side checks of the firmware with a plain C++ compiler. Use it when `pio test -e native` is not
# available; `pio test -e native` (real Unity 2.6.1) stays the reference runner for the unit tests.
#
#   firmware/tools/run_host_tests.sh            # g++ (or $CXX), -std=gnu++17
#   CXX=clang++ firmware/tools/run_host_tests.sh
#
# What it does:
#  1. every firmware/test/test_*/test_main.cpp, built with the Arduino stub force-included, so a project
#     name that collides with a real Arduino macro (bit, min, max, abs, round, B0..B7, ...) breaks the build;
#  2. src/main.cpp against the recording Arduino stub (boot order, a command session);
#  3. every header of include/ and lib/chessbot_core/src compiled ALONE (self-contained) in five include
#     setups, including ones where an SDK-style decoy config.h / pins.h / protocol.h comes first;
#  4. src/main.cpp again with the decoys first;
#  5. the firmware sources (not the tests) under -Wpedantic -Wconversion -Wsign-conversion -Wold-style-cast;
#  6. wire compatibility of the C++ core with the Python simulator (tools/wire_compat/wire_compat.py).
# All builds use -Wall -Wextra -Wshadow -Werror. Exit status: 0 when everything passed, 1 otherwise.
set -u
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cxx="${CXX:-g++}"
std="${STD:-gnu++17}"
warn="-Wall -Wextra -Wshadow -Werror"
stub="$here/tools/host_main/Arduino.h"
decoy="$here/tools/host_main/sdk_decoy"
inc="$here/include"
src="$here/lib/chessbot_core/src"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
status=0

# 1. unit-test suites (the same -I the platformio.ini sets: include/, and the library source dir)
for suite in "$here"/test/test_*/; do
  name="$(basename "$suite")"
  if ! "$cxx" -std="$std" $warn -include "$stub" \
      -I"$here/tools/host_unity" -I"$inc" -I"$src" \
      "$suite"/*.cpp "$src"/*.cpp -o "$out/$name"; then
    echo "BUILD FAILED: $name"
    status=1
    continue
  fi
  echo "== $name"
  if ! "$out/$name"; then
    status=1
  fi
done

# 2. src/main.cpp against the recording Arduino stub
if "$cxx" -std="$std" $warn -include "$stub" \
    -I"$here/tools/host_main" -I"$here/tools/host_unity" -I"$inc" -I"$src" \
    "$here/tools/host_main/boot_check.cpp" "$src"/*.cpp -o "$out/host_main"; then
  echo "== host_main (src/main.cpp against an Arduino stub)"
  "$out/host_main" || status=1
else
  echo "BUILD FAILED: host_main"
  status=1
fi

# 3. every header on its own
check_headers() {
  local variant="$1"
  shift
  for header in "$inc"/*.h "$src"/*.h; do
    printf '#include "%s"\n' "$header" > "$out/header_alone.cpp"
    if ! "$cxx" -std="$std" $warn -fsyntax-only "$@" "$out/header_alone.cpp" 2> "$out/header_alone.log"; then
      echo "HEADER NOT SELF-CONTAINED [$variant]: ${header#"$here"/}"
      sed 's/^/    /' "$out/header_alone.log" | head -6
      status=1
    fi
  done
}
echo "== headers compiled alone"
check_headers "no project include path"              -I"$src"
check_headers "SDK decoy first"                      -I"$decoy" -I"$inc" -I"$src"
check_headers "SDK decoy last"                       -I"$inc" -I"$decoy" -I"$src"
check_headers "Arduino macros, no include path"      -include "$stub" -I"$src"
check_headers "Arduino macros, SDK decoy first"      -include "$stub" -I"$decoy" -I"$inc" -I"$src"

# 4. main.cpp when an SDK-style config.h / pins.h / protocol.h is found first
if ! "$cxx" -std="$std" $warn -fsyntax-only -include "$stub" \
    -I"$decoy" -I"$here/tools/host_main" -I"$here/tools/host_unity" -I"$inc" -I"$src" \
    "$here/tools/host_main/boot_check.cpp" 2> "$out/main_decoy.log"; then
  echo "MAIN.CPP BREAKS WHEN AN SDK-STYLE HEADER COMES FIRST"
  sed 's/^/    /' "$out/main_decoy.log" | head -6
  status=1
fi

# 6. wire compatibility: the C++ core, the Python ESP32 simulator and an independent model of the SPEC 5.1
#    rules must answer the same scripts the same way (ESTOP set, ENABLE / DISABLE, repeated ESTOP). Needs python3
#    and the repository's Python packages; skipped with a notice when python3 is missing.
echo "== wire compatibility (C++ core vs Python simulator vs model)"
if command -v python3 > /dev/null 2>&1; then
  if ! python3 -I "$here/tools/wire_compat/wire_compat.py" --cxx "$cxx"; then
    status=1
  fi
else
  echo "SKIPPED: python3 not found"
fi

# 5. the firmware sources (not the tests) under stricter warnings
echo "== core and main under -Wpedantic -Wconversion -Wsign-conversion -Wold-style-cast"
if ! "$cxx" -std="$std" $warn -Wpedantic -Wconversion -Wsign-conversion -Wold-style-cast -fsyntax-only -include "$stub" \
    -I"$here/tools/host_main" -I"$inc" -I"$src" "$src"/*.cpp "$here/src/main.cpp" 2> "$out/strict.log"; then
  echo "STRICT WARNINGS IN THE FIRMWARE SOURCES"
  sed 's/^/    /' "$out/strict.log" | head -12
  status=1
fi

if [ "$status" -eq 0 ]; then echo "ALL HOST CHECKS PASSED"; else echo "HOST CHECKS FAILED"; fi
exit "$status"
