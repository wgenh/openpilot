#!/usr/bin/bash -e

# run's on tici to create a prebuilt version of a release

set -ex

TARGET_DIR=$1
SOURCE_DIR=$2

rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/panda_h7.bin.signed

cd $TARGET_DIR

VERSION=$(cat common/version.h | awk -F[\"-]  '{print $2}')
echo "#define COMMA_VERSION \"$VERSION-release\"" > common/version.h

# Build
export PYTHONPATH="$TARGET_DIR"
scons -j$(nproc)

# release panda fw
CERT=/data/pandaextra/certs/release RELEASE=1 scons -j$(nproc) panda/

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -name '__pycache__' -delete
rm -rf .sconsign.dblite Jenkinsfile release/
rm selfdrive/modeld/models/supercombo.onnx

# Mark as prebuilt release
touch prebuilt
