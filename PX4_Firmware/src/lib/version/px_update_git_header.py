#!/usr/bin/env python
from __future__ import print_function

import argparse
import os
import sys
import subprocess
import re

parser = argparse.ArgumentParser(description="""Extract version info from git and
generate a version header file. The working directory is expected to be
the root of Firmware.""")
parser.add_argument('filename', metavar='version.h', help='Header output file')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose output', default=False)
parser.add_argument('--validate', dest='validate', action='store_true',
                    help='Validate the tag format', default=False)

args = parser.parse_args()
filename = args.filename
verbose = args.verbose
validate = args.validate

try:
    fp_header = open(filename, 'r')
    old_header = fp_header.read()
except:
    old_header = ''


# Generate the header file content
header = """
/* Auto Magically Generated file */
/* Do not edit! */
#pragma once
"""


# PX4
git_tag = subprocess.check_output('git describe --always --tags --dirty'.split(),
                                  stderr=subprocess.STDOUT).decode('utf-8').strip()
if validate:
    if verbose:
        print("testing git tag: "+git_tag)
    # remove optional '-dirty' at the end
    git_tag_test = re.sub(r'-dirty$', '', git_tag)
    # remove optional -<num_commits>-g<commit_hash> at the end (in case we are not on a tagged commit)
    git_tag_test = re.sub(r'-[0-9]+-g[0-9a-fA-F]+$', '', git_tag_test)
    # now check the version format
    m = re.match(r'v([0-9]+)\.([0-9]+)\.[0-9]+([-]?rc[0-9]+)?(-beta[0-9]+)?(-[0-9]+\.[0-9]+\.[0-9]+)?$', git_tag_test)
    if m:
        # format matches, check the major and minor numbers
        major = int(m.group(1))
        minor = int(m.group(2))
        if major < 1 or (major == 1 and minor < 9):
            print("")
            print("Error: PX4 version too low, expected at least v1.9.0")
            print("Check the git tag (current tag: '{:}')".format(git_tag_test))
            print("")
            sys.exit(1)
    else:
        print("")
        print("Error: the git tag '{:}' does not match the expected format.".format(git_tag_test))
        print("")
        print("The expected format is 'v<PX4 version>[-<custom version>]'")
        print("  <PX4 version>: v<major>.<minor>.<patch>[-rc<rc>|-beta<beta>]")
        print("  <custom version>: <major>.<minor>.<patch>")
        print("Examples:")
        print("  v1.9.0rc3 (deprecated)")
        print("  v1.9.0-rc3 (preferred)")
        print("  v1.9.0-beta1")
        print("  v1.9.0-1.0.0")
        print("See also https://dev.px4.io/master/en/setup/building_px4.html#firmware_version")
        print("")
        sys.exit(1)

git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      stderr=subprocess.STDOUT).decode('utf-8').strip()
try:
    git_branch_name = subprocess.check_output('git symbolic-ref -q --short HEAD'.split(),
                                          stderr=subprocess.STDOUT).decode('utf-8').strip()
except:
    git_branch_name = ''
git_version_short = git_version[0:16]

header += """
#define PX4_GIT_VERSION_STR  "{git_version}"
#define PX4_GIT_VERSION_BINARY 0x{git_version_short}
#define PX4_GIT_TAG_STR  "{git_tag}"
#define PX4_GIT_BRANCH_NAME  "{git_branch_name}"
""".format(git_tag=git_tag,
           git_version=git_version,
           git_version_short=git_version_short,
           git_branch_name=git_branch_name)


# ECL
if (os.path.exists('src/lib/ecl/.git')):
    ecl_git_tag = subprocess.check_output('git describe --always --tags --dirty'.split(),
                                  cwd='src/lib/ecl', stderr=subprocess.STDOUT).decode('utf-8')

    ecl_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd='src/lib/ecl', stderr=subprocess.STDOUT).decode('utf-8').strip()
    ecl_git_version_short = ecl_git_version[0:16]

    header += """
#define ECL_LIB_GIT_VERSION_STR  "{ecl_git_version}"
#define ECL_LIB_GIT_VERSION_BINARY 0x{ecl_git_version_short}
""".format(ecl_git_version=ecl_git_version,
           ecl_git_version_short=ecl_git_version_short)


# Mavlink
if (os.path.exists('mavlink/include/mavlink/v2.0/.git')):
    mavlink_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd='mavlink/include/mavlink/v2.0', stderr=subprocess.STDOUT).decode('utf-8').strip()
    mavlink_git_version_short = mavlink_git_version[0:16]

    header += """
#define MAVLINK_LIB_GIT_VERSION_STR  "{mavlink_git_version}"
#define MAVLINK_LIB_GIT_VERSION_BINARY 0x{mavlink_git_version_short}
""".format(mavlink_git_version=mavlink_git_version,
           mavlink_git_version_short=mavlink_git_version_short)


# NuttX
if (os.path.exists('platforms/nuttx/NuttX/nuttx/.git')):
    nuttx_git_tag = subprocess.check_output('git describe --always --tags --match nuttx-*  --dirty'.split(),
                                  cwd='platforms/nuttx/NuttX/nuttx', stderr=subprocess.STDOUT).decode('utf-8').strip().replace("nuttx-","v")
    nuttx_git_tag = re.sub('-.*','.0',nuttx_git_tag)
    nuttx_git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(),
                                      cwd='platforms/nuttx/NuttX/nuttx', stderr=subprocess.STDOUT).decode('utf-8').strip()
    nuttx_git_version_short = nuttx_git_version[0:16]

    header += """
#define NUTTX_GIT_VERSION_STR  "{nuttx_git_version}"
#define NUTTX_GIT_VERSION_BINARY 0x{nuttx_git_version_short}
#define NUTTX_GIT_TAG_STR  "{nuttx_git_tag}"
""".format(nuttx_git_version=nuttx_git_version,
           nuttx_git_version_short=nuttx_git_version_short,
           nuttx_git_tag=nuttx_git_tag)


if old_header != header:
    if verbose:
        print('Updating header {}'.format(filename))
    fp_header = open(filename, 'w')
    fp_header.write(header)
