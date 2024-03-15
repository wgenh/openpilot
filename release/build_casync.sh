#!/usr/bin/bash

set -ex

RELEASE_CHANNEL=$1

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

CASYNC_DIR="${CASYNC_DIR:=/tmp/casync}"
export SOURCE_DIR="$(git -C $DIR rev-parse --show-toplevel)"
export TARGET_DIR="${TARGET_DIR:=$(mktemp -d)}"

echo "Creating casync release from $SOURCE_DIR to $TARGET_DIR with caidx file in $CASYNC_DIR"

mkdir -p $CASYNC_DIR
mkdir -p $TARGET_DIR

cp -pR --parents $(cat release/files_*) $TARGET_DIR/

release/create_prebuilt.sh $TARGET_DIR $SOURCE_DIR
release/create_casync_channel.py $TARGET_DIR $CASYNC_DIR $RELEASE_CHANNEL
