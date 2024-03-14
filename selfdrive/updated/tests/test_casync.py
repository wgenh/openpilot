import contextlib
import http
import json
import os
import pathlib

from asyncio import subprocess

from openpilot.selfdrive.test.helpers import DirectoryHttpServer, http_server_context, processes_context
from openpilot.selfdrive.updated.tests.test_base import BaseUpdateTest, run, update_release


CASYNC_ARGS = ["--with=symlinks"]



def create_manifest(release, version, agnos_version, release_notes):
  return {
    "name": release,
    "openpilot": {
      "version": version,
      "release_notes": release_notes
    }
  }


def create_remote_manifest(release, version, agnos_version, release_notes, casync_caidx, casync_digest):
  manifest = create_manifest(release, version, agnos_version, release_notes)

  manifest["casync"] = {
    "caidx": casync_caidx,
    "digest": casync_digest
  }

  return manifest

def create_casync_files(dirname, release, version, agnos_version, release_notes):
  with open(pathlib.Path(dirname) / ".caexclude", "w") as f:
    f.write("*\n")
    f.write("!channel.json\n")
    f.write("!RELEASES.md\n")
    f.write("!common\n")
    f.write("!common/version.h\n")
    f.write("!launch_env.sh\n")
    f.write("!test_symlink\n")

  with open(pathlib.Path(dirname) / "channel.json", "w") as f:
    json.dump(create_manifest(release, version, agnos_version, release_notes), f)


def create_casync_release(casync_dir, release, remote_dir):
  run(["casync", "make", *CASYNC_ARGS, casync_dir / f"{release}.caidx", remote_dir])
  digest = run(["casync", "digest", *CASYNC_ARGS, remote_dir], stdout=subprocess.PIPE).stdout.decode().strip()
  return digest


def OpenpilotChannelMockAPI(release_digests, mock_releases, casync_base):
  class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
      if self.path == "/openpilot/channels":
        response = list(release_digests.keys())
      else:
        channel = self.path.split("/")[-1]
        response = create_remote_manifest(channel, *mock_releases[channel], f"{casync_base}/{channel}.caidx", release_digests[channel])

      response = json.dumps(response)

      self.send_response(200)
      self.send_header('Content-Type', 'application/json')
      self.end_headers()
      self.wfile.write(response.encode(encoding='utf_8'))

  return Handler


class TestUpdateDCASyncStrategy(BaseUpdateTest):
  def setUp(self):
    super().setUp()
    self.casync_dir = self.mock_update_path / "casync"
    self.casync_dir.mkdir()
    self.release_digests = {}

  def update_remote_release(self, release):
    update_release(self.remote_dir, release, *self.MOCK_RELEASES[release])
    create_casync_files(self.remote_dir, release, *self.MOCK_RELEASES[release])
    self.release_digests[release] = create_casync_release(self.casync_dir, release, self.remote_dir)

  def setup_remote_release(self, release):
    self.update_remote_release(release)

  def setup_basedir_release(self, release):
    super().setup_basedir_release(release)
    update_release(self.basedir, release, *self.MOCK_RELEASES[release])
    create_casync_files(self.basedir, release, *self.MOCK_RELEASES[release])

  @contextlib.contextmanager
  def additional_context(self):
    with http_server_context(DirectoryHttpServer(self.casync_dir)) as (casync_host, casync_port):
      casync_base = f"http://{casync_host}:{casync_port}"

      with http_server_context(OpenpilotChannelMockAPI(self.release_digests, self.MOCK_RELEASES, casync_base)) as (api_host, api_port):
        os.environ["API_HOST"] = f"http://{api_host}:{api_port}"
        yield

  def setup_git_basedir_release(self, release):
    super().setup_basedir_release(release)
    run(["git", "init"], cwd=self.basedir)
    run(["git", "checkout", "-b", release], cwd=self.basedir)
    update_release(self.basedir, release, *self.MOCK_RELEASES[release])
    run(["git", "add", "."], cwd=self.basedir)
    run(["git", "commit", "-m", f"openpilot release {release}"], cwd=self.basedir)

  def test_recover_from_git_update(self):
    # starts off on a git update, ensures we can recover and install the correct update
    self.setup_git_basedir_release("release3")
    self.setup_remote_release("release3")

    with self.additional_context(), processes_context(["updated"]) as [_]:
      self._test_params("release3", False, False)
      self.wait_for_idle()
      self._test_params("release3", False, True)
