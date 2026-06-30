#!/bin/bash

# clean


SYMROS_PROJ="/home/ubuntu/symros/SYMROS"
EXEC="$SYMROS_PROJ/clang-util/build/node-field-identify"

# clean
rm -rf $SYMROS_PROJ/clang-util/result/*

if [ -z "$1" ]; then
  echo "Usage: $0 <search directory>"
  exit 1
fi

TARGET_DIR="$1"

if [ ! -x "$EXEC" ]; then
  echo "Error: executable $EXEC not found or not executable."
  exit 1
fi

find "$TARGET_DIR" -type f \( -name "*.h" -o -name "*.hpp" \) | while read -r file; do
  echo "Running: $EXEC \"$file\""
  "$EXEC" "$file"
done