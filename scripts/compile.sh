#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../" &>/dev/null && pwd)"

cd ${PROJECT_DIR}
if [ ! -e src/lib/fmt/doc ]; then
  mkdir src/lib || true
  git submodule update --init
fi

mkdir -p build
cd build/

# config
cmake ..

make -j