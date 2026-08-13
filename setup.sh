#!/bin/bash
# Fetches the CMSIS headers this project builds against.
# They are not vendored because they are large, separately licensed,
# and identical for every user. Run once after cloning.

set -e

if [ -f cmsis/Device/ST/STM32F1xx/Include/stm32f1xx.h ]; then
    echo "CMSIS already present, nothing to do."
    exit 0
fi

command -v git >/dev/null || { echo "git is required"; exit 1; }

echo "Fetching CMSIS headers..."
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

git clone --depth 1 -q https://github.com/STMicroelectronics/cmsis_device_f1 "$TMP/dev"
git clone --depth 1 -q https://github.com/ARM-software/CMSIS_5 "$TMP/core"

mkdir -p cmsis/Device/ST/STM32F1xx cmsis/Include
cp -r "$TMP/dev/Include" cmsis/Device/ST/STM32F1xx/
cp -r "$TMP/core/CMSIS/Core/Include/." cmsis/Include/

echo "Done. Now run: make"
