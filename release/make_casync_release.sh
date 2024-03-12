#!/usr/bin/bash

set -ex

CASYNC_DIR=/tmp/casync
CASYNC_ARGS="--with=symlinks"
RELEASE_CHANNEL=$1

mkdir -p $CASYNC_DIR

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

source $DIR/release_common.sh

$DIR/copy_release_files.sh release/files_common
$DIR/copy_release_files.sh release/files_tici

release/make_casync_files.py $TARGET_DIR $RELEASE_CHANNEL

echo $RELEASE_CHANNEL > $TARGET_DIR/.casync

casync make $CASYNC_DIR/$RELEASE_CHANNEL.caidx $TARGET_DIR

RELEASE_DIGEST=$(casync digest $CASYNC_ARGS $TARGET_DIR)

echo $RELEASE_DIGEST > $CASYNC_DIR/$RELEASE_CHANNEL.digest

echo "Created release at $CASYNC_DIR/$RELEASE_CHANNEL.caidx with digest $RELEASE_DIGEST"
