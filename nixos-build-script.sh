#!/bin/bash
# Simple build script for NixOS

# Create output directory
OUT_DIR=build/hot_reload
EXE=game_hot_reload.bin
mkdir -p $OUT_DIR
mkdir -p $OUT_DIR/linux

# Linux/NixOS settings
DLL_EXT=".so"
EXTRA_LINKER_FLAGS="'-Wl,-rpath=\$ORIGIN/linux'"

# Build the game DLL
echo "Building game$DLL_EXT"
odin build source -extra-linker-flags:"$EXTRA_LINKER_FLAGS" -define:RAYLIB_SHARED=true -build-mode:dll -out:$OUT_DIR/game_tmp$DLL_EXT -strict-style -vet -debug

# Rename the temporary file to the final name
mv $OUT_DIR/game_tmp$DLL_EXT $OUT_DIR/game$DLL_EXT

# Check if executable is already running
if pgrep -f $EXE > /dev/null; then
    echo "Hot reloading..."
    exit 0
fi

# Build the executable
echo "Building $EXE"
odin build source/main_hot_reload -out:$EXE -strict-style -vet -debug

# Run if requested
if [ $# -ge 1 ] && [ $1 == "run" ]; then
    echo "Running $EXE"
    ./$EXE &
fi
