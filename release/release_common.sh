#!/usr/bin/bash

export DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

export SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
if [ -z "$TARGET_DIR" ]; then
  export TARGET_DIR="$(mktemp -d)"
fi

# set git identity
source $DIR/identity.sh

rm -rf $TARGET_DIR
mkdir -p $TARGET_DIR
