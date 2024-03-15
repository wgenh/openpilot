import os
import pathlib
import subprocess


def run(cmd: list[str], cwd: str = None, env = None) -> str:
  if env is None:
    env = os.environ
  return subprocess.check_output(cmd, cwd=cwd, stderr=subprocess.STDOUT, encoding='utf8', env=env)


def get_git_branch(path: str) -> str | None:
  return run(["git", "rev-parse", "--abbrev-ref", "HEAD"], path).rstrip()


def get_version(path: str) -> str:
  with open(os.path.join(path, "common", "version.h")) as f:
    return f.read().split('"')[1]


def get_release_notes(path: str) -> str:
  with open(os.path.join(path, "RELEASES.md"), "r") as f:
    return f.read().split('\n\n', 1)[0]


def get_consistent_flag(path: str) -> bool:
  return pathlib.Path(os.path.join(path, ".overlay_consistent")).is_file()
