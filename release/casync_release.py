#!/usr/bin/env python

import argparse
import json
import pathlib
import subprocess

from openpilot.selfdrive.updated.casync import CASYNC_ARGS, CHANNEL_MANIFEST_FILE
from openpilot.selfdrive.updated.common import get_release_notes, get_version

CASYNC_FILES = [CHANNEL_MANIFEST_FILE, ".caexclude"]


def run(cmd):
  return subprocess.check_output(cmd)


def get_exclude_set(path) -> set[str]:
  exclude_set = set(CASYNC_FILES)

  for file in path.rglob("*"):
    if file.is_file() or file.is_symlink():

      while file.resolve() != path.resolve():
        exclude_set.add(str(file.relative_to(path)))

        file = file.parent

  return exclude_set


def create_caexclude_file(path: pathlib.Path):
  with open(path / ".caexclude", "w") as f:
    f.write("*\n") # exclude everything except the paths already in the release
    f.write(".*\n") # exclude everything except the paths already in the release

    for file in sorted(get_exclude_set(path)):
      f.write(f"!{file}\n")


def create_manifest(channel, version, release_notes):
  return {
    "name": channel,
    "openpilot": {
      "version": version,
      "release_notes": release_notes
    }
  }

def create_manifest_file(path: pathlib.Path, channel: str):
  version = get_version(str(path))
  release_notes = get_release_notes(str(path))
  with open(path / CHANNEL_MANIFEST_FILE, "w") as f:
    f.write(json.dumps(create_manifest(channel, version, release_notes)))


def create_casync_release(output_dir: pathlib.Path, target_dir: pathlib.Path, release: str):
  caidx_file = output_dir / f"{release}.caidx"
  run(["casync", "make", *CASYNC_ARGS, caidx_file, target_dir])
  digest = run(["casync", "digest", *CASYNC_ARGS, target_dir]).decode("utf-8").strip()
  return digest, caidx_file


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="creates a casync release")
  parser.add_argument("target_dir", type=str, help="target directory to build release from")
  parser.add_argument("output_dir", type=str, help="output directory for the release")
  parser.add_argument("channel", type=str, help="what channel this build is")
  args = parser.parse_args()

  target_dir = pathlib.Path(args.target_dir)
  output_dir = pathlib.Path(args.output_dir)

  create_caexclude_file(target_dir)
  create_manifest_file(target_dir, args.channel)

  digest, caidx = create_casync_release(target_dir, output_dir, args.channel)

  print(f"Created casync release from {target_dir} to {caidx} with digest {digest}")
