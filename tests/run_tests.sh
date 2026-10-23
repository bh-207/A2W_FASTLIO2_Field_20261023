#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)

python3 "$repository_root/tests/test_runtime_config.py" -v
python3 "$repository_root/tests/test_runtime_user_guide.py" -v
python3 "$repository_root/tests/test_stage_contracts.py" -v
python3 "$repository_root/tests/test_navigation_contract.py" -v
PYTHONPATH="$repository_root/src/a2w_nav_map${PYTHONPATH:+:$PYTHONPATH}" \
    /usr/bin/python3 -m unittest discover -s "$repository_root/src/a2w_nav_map/test" -v
bash "$repository_root/tests/test_portable_launchers.sh"
bash "$repository_root/tests/test_repository_hygiene.sh"
bash "$repository_root/tests/test_full_validation_runner.sh"
