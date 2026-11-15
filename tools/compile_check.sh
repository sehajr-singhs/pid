#!/usr/bin/env bash
# Compile runtime/ftc/SpoolCalibrate.java against hand-written FTC SDK stubs.
# The real SDK is not installed here, so this proves syntax + member usage only.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=tools/build/stubs
rm -rf "$OUT" && mkdir -p "$OUT"
find tools/ftc-sdk-stubs -name '*.java' > tools/build/stub_sources.txt
javac -d "$OUT" @tools/build/stub_sources.txt
javac -cp "$OUT" -d "$OUT" spool/runtime/ftc/SpoolCalibrate.java
echo "OK: SpoolCalibrate.java compiles against the SDK stubs"
