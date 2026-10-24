#!/usr/bin/env bash
# Build and run every firmware/test/test_*/test_main.cpp with a plain host C++ compiler and the tiny
# Unity-compatible header in tools/host_unity. Use it when `pio test -e native` is not available.
# `pio test -e native` (real Unity 2.6.1) is the reference runner; this script runs the same sources.
#
#   firmware/tools/run_host_tests.sh            # g++ (or $CXX), -std=gnu++17
#   CXX=clang++ firmware/tools/run_host_tests.sh
#
# Exit status: 0 when every suite passed, 1 otherwise.
set -u
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cxx="${CXX:-g++}"
std="${STD:-gnu++17}"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
status=0
for suite in "$here"/test/test_*/; do
  name="$(basename "$suite")"
  if ! "$cxx" -std="$std" -Wall -Wextra -Werror \
      -I"$here/tools/host_unity" -I"$here/include" -I"$here/lib/chessbot_core/src" \
      "$suite"/*.cpp "$here"/lib/chessbot_core/src/*.cpp -o "$out/$name"; then
    echo "BUILD FAILED: $name"
    status=1
    continue
  fi
  echo "== $name"
  if ! "$out/$name"; then
    status=1
  fi
done
# src/main.cpp itself, against a recording Arduino stub: boot order and a command session.
if "$cxx" -std="$std" -Wall -Wextra -Werror \
    -I"$here/tools/host_main" -I"$here/tools/host_unity" -I"$here/include" -I"$here/lib/chessbot_core/src" \
    "$here/tools/host_main/boot_check.cpp" "$here"/lib/chessbot_core/src/*.cpp -o "$out/host_main"; then
  echo "== host_main (src/main.cpp against an Arduino stub)"
  "$out/host_main" || status=1
else
  echo "BUILD FAILED: host_main"
  status=1
fi
if [ "$status" -eq 0 ]; then echo "ALL HOST SUITES PASSED"; else echo "HOST SUITES FAILED"; fi
exit "$status"
