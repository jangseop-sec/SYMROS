#!/bin/bash
# Clean & rebuild ONLY the symros_exec package in the three workspaces.
# For each workspace: remove build/symros_exec and install/symros_exec, then
# rebuild symros_exec with that workspace's build script (--packages-select).
#
# Usage: ./samples/rebuild_symros_exec.bash

SYMROS_PROJ=/home/ubuntu/symros/SYMROS
WS_ROOT=/home/ubuntu/symros/workspace
SCRIPTS=$SYMROS_PROJ/scripts

# workspace -> build script (matches how each is actually built)
WORKSPACES=(conc_ws sym_ws chk_ws)
declare -A BUILD=(
  [conc_ws]=conc_build.bash
  [sym_ws]=sym_build.bash
  [chk_ws]=sym_build.bash
)

declare -A RESULT

run_ws() {
  local ws=$1
  local script=${BUILD[$ws]}
  local wspath=$WS_ROOT/$ws
  echo "==================== $ws ($script) ===================="
  echo "[clean] rm -rf $wspath/build/symros_exec $wspath/install/symros_exec"
  rm -rf "$wspath/build/symros_exec" "$wspath/install/symros_exec"
  echo "[build] $SCRIPTS/$script $wspath symros_exec select"
  bash "$SCRIPTS/$script" "$wspath" symros_exec select
  RESULT[$ws]=$?
}

for ws in "${WORKSPACES[@]}"; do
  run_ws "$ws"
done

echo
echo "==================== summary ===================="
fail=0
for ws in "${WORKSPACES[@]}"; do
  if [ "${RESULT[$ws]}" -eq 0 ]; then
    echo "  $ws: OK"
  else
    echo "  $ws: FAILED (exit ${RESULT[$ws]})"
    fail=1
  fi
done
exit $fail
