#!/usr/bin/env bash
# Offline binary only: no ROS launch, robot connection, map or TF publication.
set -euo pipefail
if (( $# != 5 )); then
  echo "usage: bash $0 BINARY BUNDLE NEW_OUTPUT_DIR legacy|enhanced|scheduled|scheduled_cached PACE_0_OR_1" >&2
  exit 2
fi
binary=$(realpath "$1")
bundle=$(realpath "$2")
output=$(realpath -m "$3")
mode=$4
pace=$5
[[ -x "$binary" && -d "$bundle" ]]
[[ "$mode" == legacy || "$mode" == enhanced || "$mode" == scheduled || "$mode" == scheduled_cached ]]
[[ "$pace" == 0 || "$pace" == 1 ]]
[[ ! -e "$output" && ! -e "$output.execution" ]]
case "$output/" in "$bundle/"*) echo "output must not be in Bundle" >&2; exit 2;; esac
# A cached replay must never silently load the ABI-incompatible production common
# libraries via an inherited LD_LIBRARY_PATH. Other modes keep their old contract.
isolated_common=
if [[ "$mode" == scheduled_cached ]]; then
  isolated_common=$(realpath "${A2W_REPLAY_COMMON_LIBRARY_DIR:-$(dirname "$binary")/../install_common/lib}")
  [[ -d "$isolated_common" ]] || { echo "missing isolated common library directory" >&2; exit 2; }
  export LD_LIBRARY_PATH="$isolated_common${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export OMP_THREAD_LIMIT="${OMP_THREAD_LIMIT:-4}"
fi
evidence="$output.execution"
mkdir -p "$(dirname "$evidence")"
mkdir "$evidence"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 OMP_DYNAMIC=FALSE
export A2W_REPLAY_PACE="$pace"
unset A2W_REPLAY_SC_CACHE
{
  printf 'binary=%q\nbundle=%q\noutput=%q\nmode=%q\npace=%q\n' "$binary" "$bundle" "$output" "$mode" "$pace"
  printf 'OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 OMP_DYNAMIC=FALSE SC_CACHE=unset\n'
  printf 'OMP_THREAD_LIMIT=%s\nA2W_FROZEN_MATCHER_TIME=%s\nLD_PRELOAD=%s\n' "${OMP_THREAD_LIMIT:-unset}" "${A2W_FROZEN_MATCHER_TIME:-unset}" "${LD_PRELOAD:-unset}"
  printf 'isolated_common=%q\nLD_LIBRARY_PATH=%q\n' "$isolated_common" "${LD_LIBRARY_PATH:-}"
  date -u '+started_utc=%FT%TZ'
  uname -sm
  getconf _NPROCESSORS_ONLN
  sha256sum "$binary" "$0" "$bundle/manifest.sha256"
} > "$evidence/command_identity.txt"
ldd "$binary" > "$evidence/loaded_libraries.txt"
if [[ "$mode" == scheduled_cached ]]; then
  /usr/bin/python3 "$(dirname "$0")/replay_library_contract.py" \
    "$evidence/loaded_libraries.txt" "$isolated_common" "$evidence/runtime_library_contract.json"
fi
# Capture library content identities, not merely their potentially stale paths.
# Include direct absolute entries such as LD_PRELOAD DSOs, not only "name => path".
/usr/bin/python3 - "$evidence/loaded_libraries.txt" > "$evidence/loaded_library_sha256.txt" <<'PY'
import hashlib
from pathlib import Path
import re
import sys
pattern = re.compile(r'^\s*(?:\S+\s+=>\s+)?(/.+?)(?:\s+\(0x[0-9a-fA-F]+\))?\s*$')
paths = {Path(m.group(1)).resolve(strict=True)
         for line in Path(sys.argv[1]).read_text().splitlines()
         if (m := pattern.match(line))}
for path in sorted(paths):
    print(hashlib.sha256(path.read_bytes()).hexdigest(), str(path))
PY
"$binary" "$bundle" "$output" "$mode" > "$evidence/stdout.log" 2> "$evidence/stderr.log" &
replay_pid=$!
trap 'kill -TERM "$replay_pid" 2>/dev/null || true; wait "$replay_pid" 2>/dev/null || true' INT TERM EXIT
printf 'wall_epoch_s pid cpu_percent_since_start rss_kib observed_threads\n' > "$evidence/resources.txt"
while kill -0 "$replay_pid" 2>/dev/null; do
  sample=$(ps -p "$replay_pid" -o pid=,pcpu=,rss=,nlwp= || true)
  if [[ -n "$sample" ]]; then
    printf '%s %s\n' "$(date +%s.%N)" "$sample" >> "$evidence/resources.txt"
  fi
  sleep 1
done
status=0
wait "$replay_pid" || status=$?
trap - INT TERM EXIT
printf '%s\n' "$status" > "$evidence/exit_code.txt"
date -u '+finished_utc=%FT%TZ' >> "$evidence/command_identity.txt"
exit "$status"
