#!/usr/bin/env bash
# 根目录快捷入口，转发至 src/vio/scripts/run_vio_live.sh（避免两份脚本分叉）。
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$REPO_ROOT/src/vio/scripts/run_vio_live.sh" "$@"
