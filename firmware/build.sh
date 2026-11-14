#!/usr/bin/env bash
# Build the T-Watch S3 firmware; with a port, also flash it over USB:  firmware/build.sh [/dev/ttyACM0]
# The sketch is twatch/ plus the core (firmware/core), put together in .cache (Arduino builds one folder).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLI="${ARDUINO_CLI:-$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")}"
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,CDCOnBoot=cdc,CPUFreq=80"
# the secret half of the key (hw.cpp chip_key): made once, kept out of git
SALT="$ROOT/firmware/twatch/key_salt.h"
[ -f "$SALT" ] || echo "#define FHS_KEY_SALT 0x$(od -An -N8 -tx8 /dev/urandom | tr -d ' ')ULL" >"$SALT"
SK="$ROOT/.cache/twatch/twatch"
rm -rf "$SK"
mkdir -p "$SK/src" "$ROOT/firmware/out/twatch"
cp "$ROOT"/firmware/twatch/{twatch.ino,hw.cpp,partitions.csv,prompts.h} "$SALT" "$SK/"
cp -r "$ROOT/firmware/core" "$SK/src/"
"$CLI" compile --fqbn "$FQBN" --warnings all --output-dir "$ROOT/firmware/out/twatch" "$SK"
# partitions.csv gives the firmware 1 MB (the rest is the log); the board's own limit above is 3 MB, so check here
BIN="$ROOT/firmware/out/twatch/twatch.ino.bin"
[ "$(stat -c %s "$BIN")" -le 1048576 ] || { echo "firmware is over 1 MB: it doesn't fit its partition (partitions.csv)" >&2; exit 1; }
if [ $# -ge 1 ]; then
  "$CLI" upload --fqbn "$FQBN" -p "$1" --input-dir "$ROOT/firmware/out/twatch" "$SK"
fi
