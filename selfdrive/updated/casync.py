from enum import StrEnum
import json
import os
from pathlib import Path
import shutil
import subprocess
from markdown_it import MarkdownIt
import requests

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.updated.tests.test_base import get_consistent_flag
from openpilot.selfdrive.updated.updated import UserRequest, WaitTimeHelper

UPDATE_DELAY = 60
CHANNELS_API_ROOT = "openpilot/channels"

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')

def get_available_channels():
  return requests.get(f"{API_HOST}/{CHANNELS_API_ROOT}").json()

def get_remote_manifest(channel):
  return requests.get(f"{API_HOST}/{CHANNELS_API_ROOT}/{channel}").json()

LOCK_FILE = os.getenv("UPDATER_LOCK_FILE", "/tmp/safe_staging_overlay.lock")
STAGING_ROOT = os.getenv("UPDATER_STAGING_ROOT", "/data/safe_staging")

CASYNC_PATH = Path(STAGING_ROOT) / "casync"        # where the casync update is pulled
CASYNC_TMPDIR = Path(STAGING_ROOT) / "casync_tmp"  # working directory for casync temp files
FINALIZED = os.path.join(STAGING_ROOT, "finalized")

CASYNC_ARGS = ["--with=symlinks"]

CHANNEL_MANIFEST_FILE = "channel.json" # file that contains details of the current release


def read_manifest(path: str = BASEDIR) -> dict | None:
  try:
    with open(Path(path) / CHANNEL_MANIFEST_FILE) as f:
      return dict(json.load(f))
  except Exception:
    return None


def set_consistent_flag(consistent: bool) -> None:
  os.sync()
  consistent_file = Path(os.path.join(FINALIZED, ".overlay_consistent"))
  if consistent:
    consistent_file.touch()
  elif not consistent:
    consistent_file.unlink(missing_ok=True)
  os.sync()


class UpdaterState(StrEnum):
  IDLE = "idle"
  CHECKING = "checking..."
  DOWNLOADING = "downloading..."
  FINALIZING = "finalizing update..."


def set_status_params(state: UpdaterState = UpdaterState.CHECKING, update_available = False, update_ready = False):
  params = Params()
  params.put("UpdaterState", state)
  params.put_bool("UpdaterFetchAvailable", update_available)
  params.put_bool("UpdateAvailable", update_ready)


def set_channel_params(name, manifest):
  params = Params()
  params.put(f"Updater{name}Description", f'{manifest["openpilot"]["version"]} / {manifest["name"]}')
  params.put(f"Updater{name}ReleaseNotes", bytes(MarkdownIt().render(manifest["openpilot"]["release_notes"]), encoding="utf-8"))


def set_current_channel_params(manifest):
  set_channel_params("Current", manifest)


def set_new_channel_params(manifest):
  set_channel_params("New", manifest)


def get_digest(directory) -> str | None:
  return str(run(["casync", "digest", *CASYNC_ARGS, directory])).strip()


def check_update_available(current_directory, other_manifest):
  return read_manifest(current_directory)["name"] != other_manifest["name"] or \
         get_digest(current_directory) != other_manifest["casync"]["digest"]


def run(cmd: list[str], cwd: str = None, env = None) -> str:
  if env is None:
    env = os.environ
  return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, encoding='utf8', env=env)


def download_update(manifest):
  cloudlog.info("")
  env = os.environ.copy()
  env["TMPDIR"] = str(CASYNC_TMPDIR)
  CASYNC_TMPDIR.mkdir(exist_ok=True)
  CASYNC_PATH.mkdir(exist_ok=True)
  run(["casync", "extract", manifest["casync"]["caidx"], str(CASYNC_PATH), f"--seed={BASEDIR}", *CASYNC_ARGS], env=env)


def finalize_update():
  # Remove the update ready flag and any old updates
  cloudlog.info("creating finalized version of the overlay")
  set_consistent_flag(False)

  # Copy the merged overlay view and set the update ready flag
  if os.path.exists(FINALIZED):
    shutil.rmtree(FINALIZED)
  shutil.copytree(CASYNC_PATH, FINALIZED, symlinks=True)

  set_consistent_flag(True)
  cloudlog.info("done finalizing overlay")


def main():
  params = Params()
  set_status_params()

  current_manifest = read_manifest(BASEDIR)
  params.put("UpdaterTargetBranch", current_manifest["name"])

  wait_helper = WaitTimeHelper()

  while True:
    wait_helper.ready_event.clear()

    target_channel = params.get("UpdaterTargetBranch", encoding='utf8')

    user_requested_check = wait_helper.user_request == UserRequest.CHECK

    set_status_params(UpdaterState.CHECKING)

    update_ready = get_consistent_flag(FINALIZED)

    current_manifest = read_manifest(BASEDIR)

    set_current_channel_params(current_manifest)

    remote_manifest = get_remote_manifest(target_channel)

    update_available = check_update_available(BASEDIR, remote_manifest)

    if update_ready and not check_update_available(FINALIZED, remote_manifest):
      update_available = False

    set_status_params(UpdaterState.IDLE, update_available, update_ready)

    if update_available:
      if user_requested_check:
        cloudlog.info("skipping fetch, only checking")
      else:
        update_available = False
        set_status_params(UpdaterState.DOWNLOADING)
        download_update(remote_manifest)

        set_status_params(UpdaterState.FINALIZING)
        finalize_update()
        new_manifest = read_manifest(FINALIZED)
        set_new_channel_params(new_manifest)
        update_ready = get_consistent_flag(FINALIZED)

    set_status_params(UpdaterState.IDLE, update_available, update_ready)

    wait_helper.user_request = UserRequest.NONE
    wait_helper.sleep(UPDATE_DELAY)


if __name__ == "__main__":
  main()
