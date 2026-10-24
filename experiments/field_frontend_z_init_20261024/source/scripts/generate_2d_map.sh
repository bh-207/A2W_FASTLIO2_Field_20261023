#!/usr/bin/env bash
set -euo pipefail
repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
python_executable=${A2W_PYTHON_EXECUTABLE:-/usr/bin/python3}
if [[ -f ${A2W_INSTALL_SETUP:-"$repository_root/install/setup.bash"} ]]; then
    set +u
    source "${A2W_INSTALL_SETUP:-"$repository_root/install/setup.bash"}"
    set -u
fi
export PYTHONPATH="$repository_root/src/a2w_nav_map${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_executable" -m a2w_nav_map.cli "$@"
