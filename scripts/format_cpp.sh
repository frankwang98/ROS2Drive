#!/usr/bin/env bash
# Format or verify only handwritten C/C++ sources. Generated ROS artifacts and
# colcon build/install/log directories are intentionally outside this scope.
set -euo pipefail

mode="format"
if [[ "${1:-}" == "--check" ]]; then
  mode="check"
elif [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--check]" >&2
  exit 64
fi

if ! command -v clang-format >/dev/null 2>&1; then
  echo "clang-format is required (for Ubuntu: sudo apt install clang-format)" >&2
  exit 127
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mapfile -d '' sources < <(find "$root/include" "$root/src" "$root/test" \
  -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' \
              -o -name '*.hpp' -o -name '*.hh' -o -name '*.h' \) -print0 2>/dev/null)

if [[ ${#sources[@]} -eq 0 ]]; then
  exit 0
fi

if [[ "$mode" == "check" ]]; then
  clang-format --dry-run --Werror --style=file "${sources[@]}"
  echo "clang-format check passed (${#sources[@]} files)"
else
  clang-format -i --style=file "${sources[@]}"
  echo "formatted ${#sources[@]} C/C++ files"
fi
