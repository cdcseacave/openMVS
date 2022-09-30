#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
This script helps with OpenMVS scalable pipeline.

Starting from a SfM solution stored into a MVS project accompanied by the undistorted images,
the fist step is to compute all depth maps without fusion:

DensifyPointCloud scene.mvs --fusion-mode 1

Next split the scene in sub-scenes using the area parameter, which is related to the inverse of GSD;
it is a bit non intuitive, but normally it should remain constant for a desired memory limit;
for example you can use the bellow value to limit memory usage to ~16GB:

DensifyPointCloud scene.mvs --sub-scene-area 660000

disable depth-maps re-filtering by creating a file Densify.ini with just this line:

Optimize = 0

and call fusion on each of the sub-scenes like:

DensifyPointCloud scene_0000.mvs --dense-config-file Densify.ini
............
DensifyPointCloud scene_000n.mvs --dense-config-file Densify.ini

This script helps to automate the process of calling DensifyPointCloud/ReconstructMesh on all sub-scenes.

usage: MvsScalablePipeline.py openMVS_module input_scene <options>

ex: MvsScalablePipeline.py DensifyPointCloud scene_XXXX.mvs --number-views-fuse 2
"""

import os
import subprocess
import sys
import argparse
import glob

DEBUG = False

if sys.platform.startswith('win'):
    PATH_DELIM = ';'
    FOLDER_DELIM = '\\'
else:
    PATH_DELIM = ':'
    FOLDER_DELIM = '/'


def whereis(afile):
    """
    return directory in which afile is, None if not found. Look in PATH
    """
    if sys.platform.startswith('win'):
        cmd = "where"
    else:
        cmd = "which"
    try:
        ret = subprocess.run([cmd, afile], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True)
        return os.path.split(ret.stdout.decode())[0]
    except subprocess.CalledProcessError:
        return None


def find(afile):
    """
    As whereis look only for executable on linux, this find look for all file type
    """
    for d in os.environ['PATH'].split(PATH_DELIM):
        if os.path.isfile(os.path.join(d, afile)):
            return d
    return None

# Try to find openMVS binaries in PATH
OPENMVS_BIN = whereis("ReconstructMesh")

# Ask user for openMVS directory if not found
if not OPENMVS_BIN:
    OPENMVS_BIN = input("openMVS binary folder?\n")


# HELPERS for terminal colors
BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)
NO_EFFECT, BOLD, UNDERLINE, BLINK, INVERSE, HIDDEN = (0, 1, 4, 5, 7, 8)


# from Python cookbook, #475186
def has_colours(stream):
    '''
    Return stream colours capability
    '''
    if not hasattr(stream, "isatty"):
        return False
    if not stream.isatty():
        return False  # auto color only on TTYs
    try:
        import curses
        curses.setupterm()
        return curses.tigetnum("colors") > 2
    except Exception:
        # guess false in case of error
        return False

HAS_COLOURS = has_colours(sys.stdout)


def printout(text, colour=WHITE, background=BLACK, effect=NO_EFFECT):
    """
    print() with colour
    """
    if HAS_COLOURS:
        seq = "\x1b[%d;%d;%dm" % (effect, 30+colour, 40+background) + text + "\x1b[0m"
        sys.stdout.write(seq+'\r\n')
    else:
        sys.stdout.write(text+'\r\n')


# store config and data in
class ConfContainer:
    """
    Container for all the config variables
    """
    def __init__(self):
        pass

CONF = ConfContainer()

# ARGS
PARSER = argparse.ArgumentParser(
    formatter_class=argparse.RawTextHelpFormatter,
    description="Scalable MVS reconstruction with these steps: \r\n" +
    "MvsScalablePipeline.py openMVS_module input_scene <options>\r\n"
    )
PARSER.add_argument('openMVS_module',
                    help="the OpenMVS module to use: DensifyPointCloud, ReconstructMesh, etc.")
PARSER.add_argument('input_scene',
                    help="the scene name reg to process: scene_XXXX.mvs")
PARSER.add_argument('passthrough', nargs=argparse.REMAINDER, help="Option to be passed to command lines")

PARSER.parse_args(namespace=CONF)  # store args in the ConfContainer

suffix = os.path.basename(CONF.input_scene).replace('scene_XXXX','')
CONF.input_scene = CONF.input_scene.replace('_dense','').replace('_mesh','').replace('_refine','').replace('_texture','')

# Absolute path for input directory
if len(CONF.input_scene) < 10 or CONF.input_scene[-9:] != '_XXXX.mvs':
    sys.exit("%s: invalid scene name" % CONF.input_scene)

match CONF.openMVS_module:
  case 'ReconstructMesh':
    moduleSuffix = '_mesh.mvs'
  case 'RefineMesh':
    moduleSuffix = '_refine.mvs'
  case 'TextureMesh':
    moduleSuffix = '_texture.mvs'
  case _:
    moduleSuffix = '_dense.mvs'

printout("# Module {} start #".format(CONF.openMVS_module), colour=RED, effect=BOLD)
for scene_name in glob.glob(os.path.abspath(os.path.join(os.path.dirname(CONF.input_scene), 'scene_[0-9][0-9][0-9][0-9]'+suffix))):
  if os.path.exists(os.path.splitext(scene_name)[0] + moduleSuffix) == False:
    printout("# Process: %s" % os.path.basename(scene_name), colour=GREEN, effect=NO_EFFECT)

    # create a commandline for the current step
    cmdline = [os.path.join(OPENMVS_BIN, CONF.openMVS_module), scene_name] + CONF.passthrough
    print('Cmd: ' + ' '.join(cmdline))

    if not DEBUG:
        # Launch the current step
        try:
            pStep = subprocess.Popen(cmdline)
            pStep.wait()
            if pStep.returncode != 0:
                printout("# Warning: step failed", colour=RED, effect=BOLD)
        except KeyboardInterrupt:
            sys.exit('\r\nProcess canceled by user, all files remains')
    else:
        print('\t'.join(cmdline))

printout("# Module {} end #".format(CONF.openMVS_module), colour=RED, effect=BOLD)
