#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
This script is for comparing the poses stored in an OpenMVS project to the poses optimized by OpenMVG

usage: run 'MvgOptimizeSfM.py' in a sub-folder to the OpenMVS project folder containing
    'scene.mvs' and images stored in 'images' folder; structure ex:

    -OpenMVS_project
      -images
      -scene.mvs
      -mvg
        -run script here
"""

import os
import sys
import subprocess

if sys.platform.startswith('win'):
    PATH_DELIM = ';'
else:
    PATH_DELIM = ':'

# add this script's directory to PATH
os.environ['PATH'] += PATH_DELIM + os.path.dirname(os.path.abspath(__file__))

# add current directory to PATH
os.environ['PATH'] += PATH_DELIM + os.getcwd()


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


def launch(cmdline):
    # Launch the current step
    print('Cmd: ' + ' '.join(cmdline))
    try:
        pStep = subprocess.Popen(cmdline)
        pStep.wait()
        if pStep.returncode != 0:
            return
    except KeyboardInterrupt:
        sys.exit('\r\nProcess canceled by user, all files remains')


# Try to find openMVG and openMVS binaries in PATH
OPENMVG_BIN = whereis("openMVG_main_SfMInit_ImageListing")
OPENMVS_BIN = whereis("ReconstructMesh")

# Ask user for openMVG and openMVS directories if not found
if not OPENMVG_BIN:
    OPENMVG_BIN = input("openMVG binary folder?\n")
if not OPENMVS_BIN:
    OPENMVS_BIN = input("openMVS binary folder?\n")

launch([os.path.join(OPENMVS_BIN, 'InterfaceCOLMAP'), '../scene.mvs', '-o', '../gt_dense_cameras.camera'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_SfMInit_ImageListingFromKnownPoses'), '-i', '../images', '-g', '../gt_dense_cameras', '-t', '1', '-o', '.'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_ComputeFeatures'), '-i', 'sfm_data.json', '-o', '.'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_ComputeMatches'), '-i', 'sfm_data.json', '-o', '.', '-m', '1'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_ComputeStructureFromKnownPoses'), '-i', 'sfm_data.json', '-m', '.', '-o', 'sfm_data_struct.bin', '-b'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_ComputeSfM_DataColor'), '-i', 'sfm_data_struct.bin', '-o', 'scene.ply'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_openMVG2openMVS'), '-i', 'sfm_data_struct.bin', '-o', 'scene.mvs', '-d', 'images'])
launch([os.path.join(OPENMVS_BIN, 'InterfaceCOLMAP'), 'scene.mvs', '-o', 'cameras.camera'])
launch([os.path.join(OPENMVG_BIN, 'openMVG_main_evalQuality'), '-i', '..', '-c', 'sfm_data_struct.bin', '-o', 'compare'])
