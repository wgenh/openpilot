#!/usr/bin/bash

set -ex

CASYNC_DIR=/tmp/casync
RELEASE_CHANNEL=$1

mkdir -p $CASYNC_DIR

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

export SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
if [ -z "$TARGET_DIR" ]; then
  export TARGET_DIR="$(mktemp -d)"
fi

mkdir -p $TARGET_DIR

cp -pR --parents $(cat release/files_*) $TARGET_DIR/

release/create_casync_channel.py $TARGET_DIR $CASYNC_DIR $RELEASE_CHANNEL
release/create_prebuilt.sh $TARGET_DIR $SOURCE_DIR
