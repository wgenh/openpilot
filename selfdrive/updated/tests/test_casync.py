import contextlib
import os

from asyncio import subprocess
import pathlib
from unittest import mock

from openpilot.selfdrive.test.helpers import DirectoryHttpServer, http_server_context
from openpilot.selfdrive.updated.tests.test_base import BaseUpdateTest, run, update_release


CASYNC_ARGS = ["--with=symlinks"]


def create_casync_files(dirname, release):
  with open(pathlib.Path(dirname) / ".caexclude", "w") as f:
    f.write(".git\n")
    f.write(".overlay_consistent\n")
    f.write(".overlay_init\n")

  with open(pathlib.Path(dirname) / "openpilot-release.json", "w") as f:
    f.write(f'{{"channel": "{release}"}}')


def create_casync_release(casync_dir, release, remote_dir):
  run(["casync", "make", *CASYNC_ARGS, casync_dir / f"{release}.caidx", remote_dir])

  digest = run(["casync", "digest", *CASYNC_ARGS, remote_dir], stdout=subprocess.PIPE).stdout.decode().strip()

  with open(casync_dir / f"{release}.digest", "w") as f:
    f.write(digest)


class TestUpdateDCASyncStrategy(BaseUpdateTest):
  def setUp(self):
    super().setUp()
    self.casync_dir = self.mock_update_path / "casync"
    self.casync_dir.mkdir()
    os.environ["UPDATER_STRATEGY"] = "casync"

  def update_remote_release(self, release):
    update_release(self.remote_dir, release, *self.MOCK_RELEASES[release])
    create_casync_files(self.remote_dir, release)
    create_casync_release(self.casync_dir, release, self.remote_dir)

  def setup_remote_release(self, release):
    self.update_remote_release(release)

  def setup_basedir_release(self, release):
    super().setup_basedir_release(release)
    update_release(self.basedir, release, *self.MOCK_RELEASES[release])
    create_casync_files(self.basedir, release)

  @contextlib.contextmanager
  def additional_context(self):
    with http_server_context(DirectoryHttpServer(self.casync_dir)) as (host, port):
      with mock.patch("openpilot.selfdrive.updated.casync.CHANNEL_PATH", f"http://{host}:{port}"):
        yield
