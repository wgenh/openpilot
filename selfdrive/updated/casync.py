import json
import os
from pathlib import Path
import shutil
import requests
from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.updated.common import FINALIZED, STAGING_ROOT, UpdateStrategy, \
                                               get_consistent_flag, get_release_notes, get_version, run, set_consistent_flag
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.updated.git import OVERLAY_INIT, GitUpdateStrategy


CHANNEL_PATH = "https://commadist.blob.core.windows.net/openpilot-channels"

CHANNELS = {
  "test1": "test1",
  "test2": "test2",
}

CASYNC_PATH = Path(STAGING_ROOT) / "casync"
CASYNC_TMPDIR = Path(STAGING_ROOT) / "casync_tmp"

CASYNC_ARGS = ["--with=symlinks"]


RELEASE_METADATA_FILE = "openpilot-release.json"


def read_release_metadata(path: str = BASEDIR) -> dict | None:
  try:
    with open(Path(path) / RELEASE_METADATA_FILE) as f:
      return dict(json.load(f))
  except Exception:
    return None


class CASyncUpdateStrategy(UpdateStrategy):
  def init(self):
    run(["sudo", "rm", "-rf", STAGING_ROOT])
    if os.path.isdir(STAGING_ROOT):
      shutil.rmtree(STAGING_ROOT)

    for dirname in [STAGING_ROOT, CASYNC_PATH]:
      os.mkdir(dirname, 0o755)

    OVERLAY_INIT.touch()

  def get_available_channels(self) -> list[str]:
    return list(CHANNELS.keys())

  def get_digest_local(self, path: str) -> str:
    return run(["casync", "digest", *CASYNC_ARGS, path]).strip()

  def get_digest_remote(self, channel: str) -> str:
    return requests.get(f"{CHANNEL_PATH}/{channel}.digest").text.strip()

  def get_channel(self, path: str) -> str:
    release_metadata = read_release_metadata(path)

    if release_metadata is not None:
      return str(release_metadata["channel"])

    cloudlog.exception("failed to get channel from release metadata, trying to get channel from git...")

    try:
      return GitUpdateStrategy.get_branch(path)
    except Exception:
      cloudlog.exception("casync.get_channel git")

    raise Exception("unable to determine channel...")

  def current_channel(self) -> str:
    return self.get_channel(str(BASEDIR))

  def update_ready(self) -> bool:
    if get_consistent_flag():
      return self.get_digest_local(FINALIZED) != self.get_digest_local(BASEDIR)
    return False

  def update_available(self) -> bool:
    remote_digest = self.get_digest_remote(self.target_channel)

    if CASYNC_PATH.exists() and len(list(CASYNC_PATH.rglob("*"))) > 1:
      return self.get_digest_local(str(CASYNC_PATH)) != remote_digest

    return self.get_digest_local(str(BASEDIR)) != remote_digest

  def describe_channel(self, path):
    if not os.path.exists(path):
      return ""

    version = ""
    channel = ""
    try:
      channel = self.get_channel(path)
      version = get_version(path)
    except Exception:
      cloudlog.exception("casync.describe_channel")
    return f"{version} / {channel}"

  def release_notes(self, path):
    try:
      return get_release_notes(path)
    except Exception:
      cloudlog.exception("casync.release_notes")
      return ""

  def describe_current_channel(self) -> tuple[str, str]:
    return self.describe_channel(BASEDIR), self.release_notes(BASEDIR)

  def describe_ready_channel(self) -> tuple[str, str]:
    if self.update_ready():
      return self.describe_channel(FINALIZED), self.release_notes(FINALIZED)
    return "", ""

  def fetch_update(self) -> None:
    cloudlog.info("attempting a casync update inside staging path")
    env = os.environ.copy()
    env["TMPDIR"] = str(CASYNC_TMPDIR)
    CASYNC_TMPDIR.mkdir(exist_ok=True)
    run(["casync", "extract", f"{CHANNEL_PATH}/{self.target_channel}.caidx", str(CASYNC_PATH) , f"--seed={BASEDIR}", *CASYNC_ARGS], env=env)

  def fetched_path(self) -> str:
    return str(CASYNC_PATH)

  def finalize_update(self) -> None:
    # Remove the update ready flag and any old updates
    cloudlog.info("creating finalized version of the overlay")
    set_consistent_flag(False)

    # Copy the merged overlay view and set the update ready flag
    if os.path.exists(FINALIZED):
      shutil.rmtree(FINALIZED)
    shutil.copytree(CASYNC_PATH, FINALIZED, symlinks=True)

    set_consistent_flag(True)
    cloudlog.info("done finalizing overlay")

  def cleanup(self):
    pass
