#!/usr/bin/bash

# do the files copy
echo "[-] copying files from $1"
cd $SOURCE_DIR
cp -pR --parents $(cat $1) $TARGET_DIR/
