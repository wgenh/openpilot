#!/usr/bin/bash

set -ex

CASYNC_DIR=/tmp/casync
CASYNC_ARGS="--with=symlinks"
RELEASE_CHANNEL=$1

mkdir -p $CASYNC_DIR

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

source $DIR/release_common.sh

cp -pR --parents $(cat release/files_common release/files_tici) $TARGET_DIR/

release/casync_release.py $TARGET_DIR $CASYNC_DIR $RELEASE_CHANNEL
