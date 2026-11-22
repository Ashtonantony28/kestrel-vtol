#!/usr/bin/env bash
# Verify: ground/tests/run.sh  -- builds the radio-bridge core for the host and runs the unit tests,
# once plain (-O2 -Werror) and once with AddressSanitizer + UndefinedBehaviorSanitizer.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="$here/../kestrel_radio/src"
out="${TMPDIR:-/tmp}/kestrel-ground-tests"
mkdir -p "$out"
flags="-std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror"
g++ $flags -O2 -o "$out/t" "$here/test_ground.cpp" "$src/crsf.cpp" "$src/mapper.cpp"
"$out/t"
g++ $flags -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -o "$out/t_san" \
  "$here/test_ground.cpp" "$src/crsf.cpp" "$src/mapper.cpp"
"$out/t_san"
# type-check the Arduino sketch against a host stub of the Bluepad32 API
printf '#include "kestrel_radio.ino"\nint main(){setup();loop();loop();return 0;}\n' > "$out/sketch.cpp"
g++ -std=c++17 -Wall -Wextra -Werror -x c++ -I"$here/stub" -I"$here/../kestrel_radio" -o "$out/sketch" \
  "$out/sketch.cpp" "$src/crsf.cpp" "$src/mapper.cpp"
"$out/sketch"
echo "ground tests OK"
