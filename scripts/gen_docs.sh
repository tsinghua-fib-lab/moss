#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

cd ${PROJECT_DIR}
pip3 install . -v
PDOC_ALLOW_EXEC=1 pdoc -o docs/ -d markdown ./moss
# 查找目录下的所有文件，并对每个文件执行sed命令
find "docs/" -name "*.html" -type f -exec sed -i 's/moss.html/moss/g' {} +
