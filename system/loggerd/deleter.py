#!/usr/bin/env python3
import os
import shutil
import threading
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import get_directory_sort
from openpilot.system.loggerd.xattr_cache import getxattr

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE


def get_preserved_segments(dirs_by_creation: list[str]) -> list[str]:
  preserved = []
  for n, d in enumerate(filter(has_preserve_xattr, reversed(dirs_by_creation))):
    if n == PRESERVE_COUNT:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue

    # preserve segment and its prior
    preserved.append(d)
    preserved.append(f"{date_str}--{seg_num - 1}")

  return preserved


def deleter_thread(exit_event):
  while not exit_event.is_set():
    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      all_contents_in_logs_dir = os.listdir(Paths.log_root())

      dirs = [directory for directory in all_contents_in_logs_dir if os.path.isdir(os.path.join(Paths.log_root(), directory))]
      # sort directories by creation time.
      dirs = sorted(dirs, key=get_directory_sort)
      files = [file for file in all_contents_in_logs_dir if os.path.isfile(os.path.join(Paths.log_root(), file))]
      files_for_deletion = files + dirs

      # skip deleting most recent N preserved segments (and their prior segment)
      preserved_segments = get_preserved_segments(files_for_deletion)

      # remove the earliest directory we can
      for delete_item in sorted(files_for_deletion, key=lambda d: (d in DELETE_LAST, d in preserved_segments)):
        delete_path = os.path.join(Paths.log_root(), delete_item)

        if os.path.isdir(delete_path) and any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        try:
          cloudlog.info(f"deleting {delete_path}")
          if os.path.isfile(delete_path):
            os.remove(delete_path)
          else:
            shutil.rmtree(delete_path)
          break
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
      exit_event.wait(.1)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
