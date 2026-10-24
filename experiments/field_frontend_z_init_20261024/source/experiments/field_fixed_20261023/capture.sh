#!/usr/bin/env bash
set -eo pipefail
exec /usr/bin/python3 -s "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/field.py" "$@"
