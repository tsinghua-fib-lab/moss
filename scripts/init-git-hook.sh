#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# generate protobuf related .go files
cp ${PROJECT_DIR}/scripts/pre-commit ${PROJECT_DIR}/.git/hooks/pre-commit
