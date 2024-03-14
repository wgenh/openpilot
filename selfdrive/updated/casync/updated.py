from enum import StrEnum
import json
import os
from pathlib import Path
import shutil
from markdown_it import MarkdownIt
import requests

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.updated.casync.common import CASYNC_ARGS, CHANNEL_MANIFEST_FILE
from openpilot.selfdrive.updated.common import get_version, get_release_notes, get_git_branch, run
from openpilot.selfdrive.updated.tests.test_base import get_consistent_flag
from openpilot.selfdrive.updated.updated import UserRequest, WaitTimeHelper, handle_agnos_update
from openpilot.system.hardware import AGNOS

UPDATE_DELAY = 60
CHANNELS_API_ROOT = "openpilot/channels"

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')

def get_available_channels() -> list | None:
  try:
    return list(requests.get(f"{API_HOST}/{CHANNELS_API_ROOT}").json())
  except Exception:
    cloudlog.exception("fetching remote channels")
    return None

def get_remote_manifest(channel) -> dict | None:
  try:
    return dict(requests.get(f"{API_HOST}/{CHANNELS_API_ROOT}/{channel}").json())
  except Exception:
    cloudlog.exception("fetching remote manifest failed")
    return None


LOCK_FILE = os.getenv("UPDATER_LOCK_FILE", "/tmp/safe_staging_overlay.lock")
STAGING_ROOT = os.getenv("UPDATER_STAGING_ROOT", "/data/safe_staging")

CASYNC_PATH = Path(STAGING_ROOT) / "casync"        # where the casync update is pulled
CASYNC_TMPDIR = Path(STAGING_ROOT) / "casync_tmp"  # working directory for casync temp files
FINALIZED = os.path.join(STAGING_ROOT, "finalized")

def manifest_from_git(path: str) -> dict | None:
  branch = get_git_branch(path)
  version = get_version(path)
  release_notes = get_release_notes(path)

  return {
    "name": branch,
    "openpilot": {
      "version": version,
      "release_notes": release_notes
    }
  }


def read_manifest(path: str = BASEDIR) -> dict | None:
  try:
    with open(Path(path) / CHANNEL_MANIFEST_FILE) as f:
      return dict(json.load(f))
  except Exception:
    cloudlog.exception("reading channel manifest failed")

  try:
    return manifest_from_git(path)
  except Exception:
    cloudlog.exception("reading git manifest failed")

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


def set_status_params(state: UpdaterState, update_available = False, update_ready = False):
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


def download_update(manifest):
  cloudlog.info("")
  env = os.environ.copy()
  env["TMPDIR"] = str(CASYNC_TMPDIR)
  CASYNC_TMPDIR.mkdir(exist_ok=True)
  CASYNC_PATH.mkdir(exist_ok=True)
  run(["casync", "extract", manifest["casync"]["caidx"], str(CASYNC_PATH), f"--seed={BASEDIR}", *CASYNC_ARGS], env=env)


def finalize_update():
  cloudlog.info("creating finalized version of the overlay")
  set_consistent_flag(False)

  if os.path.exists(FINALIZED):
    shutil.rmtree(FINALIZED)
  shutil.copytree(CASYNC_PATH, FINALIZED, symlinks=True)

  set_consistent_flag(True)
  cloudlog.info("done finalizing overlay")


def main():
  params = Params()
  set_status_params(UpdaterState.CHECKING)

  wait_helper = WaitTimeHelper()

  while True:
    wait_helper.ready_event.clear()

    target_channel = params.get("UpdaterTargetBranch", encoding='utf8')
    current_manifest = read_manifest(BASEDIR)

    if target_channel is None:
      target_channel = current_manifest["name"]
      params.put("UpdaterTargetBranch", target_channel)

    user_requested_check = wait_helper.user_request == UserRequest.CHECK

    set_status_params(UpdaterState.CHECKING)

    update_ready = get_consistent_flag(FINALIZED)

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

        if AGNOS:
          handle_agnos_update(CASYNC_PATH)

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
