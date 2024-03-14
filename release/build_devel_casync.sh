#!/usr/bin/bash

set -ex

CASYNC_DIR=/tmp/casync
CASYNC_ARGS="--with=symlinks"
RELEASE_CHANNEL=$1

mkdir -p $CASYNC_DIR

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

export SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
if [ -z "$TARGET_DIR" ]; then
  export TARGET_DIR="$(mktemp -d)"
fi

mkdir -p $TARGET_DIR

cp -pR --parents $(cat release/files_common release/files_tici) $TARGET_DIR/

release/casync_release.py $TARGET_DIR $CASYNC_DIR $RELEASE_CHANNEL
