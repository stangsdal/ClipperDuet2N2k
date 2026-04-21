# SPDX-License-Identifier: MIT

# see https://community.platformio.org/t/how-to-build-got-revision-into-binary-for-version-output/15380/6
# and https://github.com/biologist79/ESPuino/blob/master/gitVersion.py

import pkg_resources

Import("env")

required_pkgs = {'dulwich'}
installed_pkgs = {pkg.key for pkg in pkg_resources.working_set}
missing_pkgs = required_pkgs - installed_pkgs

if missing_pkgs:
    env.Execute('"$PYTHONEXE" -m pip install dulwich')

import subprocess
import datetime
import os
from pathlib import Path

from contextlib import suppress
from dulwich import porcelain
from dulwich.errors import NotGitRepository


OUTPUT_PATH = (
    Path(env.subst("$BUILD_DIR")) / "generated" / "version.h"
)  # pylint: disable=undefined-variable

TEMPLATE = """
#ifndef __VERSION_H__
    #define __VERSION_H__
    #define N2K_SOFTWARE_VERSION "{n2k_software_version}"
    #define GIT_DESCRIBE "{git_describe}"
#endif
"""

def get_git_describe_always_dirty(repo=None):
    """# returns a version description like "git describe --always --dirty" would"""
    repo = repo or os.getcwd()
    with suppress(NotGitRepository):
        revid = None
        with porcelain.open_repo_closing(repo) as r:
            revid = r.head().decode("ascii")
        version = porcelain.describe(repo, abbrev=9)
        # version = revid[0:9] if version[0] == 'g' else version
        status = porcelain.status(repo=repo, untracked_files="no")
        dirtymark = "-dirty" if status.staged["add"] or status.staged["delete"] or status.staged["modify"] or status.unstaged else ""
        # return f"{version}~git:{revid}{dirtymark}"
        return f"{version}{dirtymark}"
    return "unknown"

def make_n2k_version(git_dirty_describe):
    """Constructs a pseudo-version string from git describe, which follows format requirements of NMEA2000"""
    g = git_dirty_describe or "unknown"

    # Default to a safe version if git describe does not start with semver.
    # This happens on repositories without a matching tag (e.g. "2bde874-dirty").
    base_version = "0.0.0"
    if g != "unknown":
        s = g[1:] if g.startswith("v") else g
        semver_token = s.split('-')[0]
        n = semver_token.split('.')
        if len(n) >= 3 and all(x.isdigit() for x in n[:3]):
            base_version = ".".join([str(int(x)) for x in n[:3]])

    p = g.split('-')
    if p[-1] == "dirty":
        return "0." + base_version

    commit_count = "0"
    if len(p) >= 2 and p[1].isdigit():
        commit_count = str(int(p[1]))

    return base_version + "." + commit_count

def get_firmware_specifier(g=None):
    """Returns a firmware version string suitable for NMEA2000 version reporting."""
    if g is None:
        g =  get_git_describe_always_dirty('.')

    x = datetime.datetime.now()
    d = x.strftime("%Y-%m-%d")

    t = make_n2k_version(g)

    n2kv = t + ' (' + d + ')'
    return n2kv

def generate():
    """Generates header file."""
    print("GENERATING GIT REVISION HEADER FILE")
    gitrev = get_git_describe_always_dirty('.')
    n2kv = get_firmware_specifier(gitrev)
    print(f'  "{gitrev}" -> {OUTPUT_PATH}')
    OUTPUT_PATH.parent.mkdir(exist_ok=True, parents=True)
    with OUTPUT_PATH.open("w") as output_file:
        output_file.write(TEMPLATE.format(n2k_software_version=n2kv, git_describe=gitrev))

# This is a convenience-function for _me_
def before_upload(source, target, env):
    # killall minicom
    if (os.environ.get("USER")=="soenke"):
        try:
            print("killall minicom")
            subprocess.run(["killall", "-u", os.environ.get("USER"), "minicom"], stdout=subprocess.PIPE, text=True)
        except FileNotFoundError:
            # killall not found
            return

generate()
env.Append(CPPPATH=OUTPUT_PATH.parent)  # pylint: disable=undefined-variable
env.AddPreAction("upload", before_upload)
