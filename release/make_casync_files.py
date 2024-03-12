#!/usr/bin/env python

import argparse
import pathlib

CASYNC_FILES = ["openpilot-release.json", ".caexclude"]


def get_exclude_set(path) -> set[str]:
  exclude_set = set(CASYNC_FILES)

  for file in path.rglob("*"):
    if file.is_file() or file.is_symlink():

      while file.resolve() != path.resolve():
        exclude_set.add(str(file.relative_to(path)))

        file = file.parent

  return exclude_set


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Generate a .caexclude file")
  parser.add_argument("path", type=str, help="path to generate caexclude from")
  parser.add_argument("channel", type=str, help="what channel this build is")
  args = parser.parse_args()

  path = pathlib.Path(args.path)

  with open(path / ".caexclude", "w") as f:
    f.write("*\n") # exclude everything except the paths already in the release
    f.write(".*\n") # exclude everything except the paths already in the release

    for file in sorted(get_exclude_set(path)):
      f.write(f"!{file}\n")

  with open(path / "openpilot-release.json", "w") as f:
    f.write(f'{{"channel": "{args.channel}"}}\n')
