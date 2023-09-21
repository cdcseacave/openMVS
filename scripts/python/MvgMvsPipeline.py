#!/usr/bin/python3
# -*- encoding: utf-8 -*-
#
# Created by @FlachyJoe
"""
This script is for an easy use of OpenMVG, COLMAP, and OpenMVS

usage: MvgMvs_Pipeline.py [-h] [--steps STEPS [STEPS ...]] [--preset PRESET]
                          [--0 0 [0 ...]] [--1 1 [1 ...]] [--2 2 [2 ...]]
                          [--3 3 [3 ...]] [--4 4 [4 ...]] [--5 5 [5 ...]]
                          [--6 6 [6 ...]] [--7 7 [7 ...]] [--8 8 [8 ...]]
                          [--9 9 [9 ...]] [--10 10 [10 ...]] [--11 11 [11 ...]]
                          [--12 12 [12 ...]] [--13 13 [13 ...]] [--14 14 [14 ...]]
                          [--15 15 [15 ...]] [--16 16 [16 ...]] [--17 17 [17 ...]]
                          [--18 18 [18 ...]] [--19 19 [19 ...]] [--20 20 [20 ...]]
                          [--21 21 [21 ...]] [--22 22 [22 ...]]
                          input_dir output_dir

Photogrammetry reconstruction with these steps:
    0. Intrinsics analysis             openMVG_main_SfMInit_ImageListing
    1. Compute features                openMVG_main_ComputeFeatures
    2. Compute pairs                   openMVG_main_PairGenerator
    3. Compute matches                 openMVG_main_ComputeMatches
    4. Filter matches                  openMVG_main_GeometricFilter
    5. Incremental reconstruction      openMVG_main_SfM
    6. Global reconstruction           openMVG_main_SfM
    7. Colorize Structure              openMVG_main_ComputeSfM_DataColor
    8. Structure from Known Poses      openMVG_main_ComputeStructureFromKnownPoses
    9. Colorized robust triangulation  openMVG_main_ComputeSfM_DataColor
    10. Control Points Registration    ui_openMVG_control_points_registration
    11. Export to openMVS              openMVG_main_openMVG2openMVS
    12. Feature Extractor              colmap
    13. Exhaustive Matcher             colmap
    14. Mapper                         colmap
    15. Image Undistorter              colmap
    16. Export to openMVS              InterfaceCOLMAP
    17. Densify point-cloud            DensifyPointCloud
    18. Reconstruct the mesh           ReconstructMesh
    19. Refine the mesh                RefineMesh
    20. Texture the mesh               TextureMesh
    21. Estimate disparity-maps        DensifyPointCloud
    22. Fuse disparity-maps            DensifyPointCloud

positional arguments:
  input_dir                 the directory which contains the pictures set.
  output_dir                the directory which will contain the resulting files.

optional arguments:
  -h, --help                show this help message and exit
  --steps STEPS [STEPS ...] steps to process
  --preset PRESET           steps list preset in
                            SEQUENTIAL = [0, 1, 2, 3, 4, 5, 11, 17, 18, 19, 20]
                            GLOBAL = [0, 1, 2, 3, 4, 6, 11, 17, 18, 19, 20]
                            MVG_SEQ = [0, 1, 2, 3, 4, 5, 7, 8, 9, 11]
                            MVG_GLOBAL = [0, 1, 2, 3, 4, 6, 7, 8, 9, 11]
                            COLMAP_MVS = [12, 13, 14, 15, 16, 17, 18, 19, 20]
                            COLMAP = [12, 13, 14, 15, 16]
                            MVS = [17, 18, 19, 20]
                            MVS_SGM = [21, 22]
                            default : SEQUENTIAL

Passthrough:
  Option to be passed to command lines (remove - in front of option names)
  e.g. --1 p ULTRA to use the ULTRA preset in openMVG_main_ComputeFeatures
  For example, running the script
  [MvgMvsPipeline.py input_dir output_dir --steps 0 1 2 3 4 5 11 17 18 20 --1 p HIGH n 8 --3 n HNSWL2]
  [--steps 0 1 2 3 4 5 11 17 18 20] runs only the desired steps
  [--1 p HIGH n 8] where --1 refer to openMVG_main_ComputeFeatures,
  p refers to describerPreset option and set to HIGH, and n refers
  to numThreads and set to 8. The second step (Compute matches),
  [--3 n HNSWL2] where --3 refer to openMVG_main_ComputeMatches,
  n refers to nearest_matching_method option and set to HNSWL2
"""

import os
import subprocess
import sys
import argparse

DEBUG = False

if sys.platform.startswith('win'):
    PATH_DELIM = ';'
    FOLDER_DELIM = '\\'
else:
    PATH_DELIM = ':'
    FOLDER_DELIM = '/'

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


def find(afile):
    """
        As whereis look only for executable on linux, this find look for all file type
    """
    for d in os.environ['PATH'].split(PATH_DELIM):
        if os.path.isfile(os.path.join(d, afile)):
            return d
    return None


# Try to find openMVG, COLMAP, and openMVS binaries in PATH
OPENMVG_BIN = whereis("openMVG_main_SfMInit_ImageListing")
COLMAP_BIN = whereis("colmap")
OPENMVS_BIN = whereis("ReconstructMesh")

# Try to find openMVG camera sensor database
CAMERA_SENSOR_DB_FILE = "sensor_width_camera_database.txt"
CAMERA_SENSOR_DB_DIRECTORY = find(CAMERA_SENSOR_DB_FILE)

# Ask user for openMVG, COLMAP, and openMVS directories if not found
if not OPENMVG_BIN:
    OPENMVG_BIN = input("openMVG binary folder?\n")
if not COLMAP_BIN:
    COLMAP_BIN = input("COLMAP binary folder?\n")
if not OPENMVS_BIN:
    OPENMVS_BIN = input("openMVS binary folder?\n")
if not CAMERA_SENSOR_DB_DIRECTORY:
    CAMERA_SENSOR_DB_DIRECTORY = input("openMVG camera database (%s) folder?\n" % CAMERA_SENSOR_DB_FILE)
COLMAP_BIN = os.path.join(COLMAP_BIN, "colmap")
if sys.platform.startswith('win'):
    COLMAP_BIN += ".bat"

PRESET = {'SEQUENTIAL': [0, 1, 2, 3, 4, 5, 11, 17, 18, 19, 20],
          'GLOBAL': [0, 1, 2, 3, 4, 6, 11, 17, 18, 19, 20],
          'MVG_SEQ': [0, 1, 2, 3, 4, 5, 7, 8, 9, 11],
          'MVG_GLOBAL': [0, 1, 2, 3, 4, 6, 7, 8, 9, 11],
          'COLMAP_MVS': [12, 13, 14, 15, 16, 17, 18, 19, 20],
          'COLMAP': [12, 13, 14, 15, 16],
          'MVS': [17, 18, 19, 20],
          'MVS_SGM': [21, 22]}

PRESET_DEFAULT = 'SEQUENTIAL'

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


# OBJECTS to store config and data in
class ConfContainer:
    """
        Container for all the config variables
    """
    def __init__(self):
        pass


class AStep:
    """ Represents a process step to be run """
    def __init__(self, info, cmd, opt):
        self.info = info
        self.cmd = cmd
        self.opt = opt


class StepsStore:
    """ List of steps with facilities to configure them """
    def __init__(self):
        self.steps_data = [
            ["Intrinsics analysis",          # 0
             os.path.join(OPENMVG_BIN, "openMVG_main_SfMInit_ImageListing"),
             ["-i", "%input_dir%", "-o", "%matches_dir%", "-d", "%camera_file_params%"]],
            ["Compute features",             # 1
             os.path.join(OPENMVG_BIN, "openMVG_main_ComputeFeatures"),
             ["-i", "%matches_dir%"+FOLDER_DELIM+"sfm_data.json", "-o", "%matches_dir%", "-m", "SIFT"]],
            ["Compute pairs",                # 2
             os.path.join(OPENMVG_BIN, "openMVG_main_PairGenerator"),
             ["-i", "%matches_dir%"+FOLDER_DELIM+"sfm_data.json", "-o", "%matches_dir%"+FOLDER_DELIM+"pairs.bin"]],
            ["Compute matches",              # 3
             os.path.join(OPENMVG_BIN, "openMVG_main_ComputeMatches"),
             ["-i", "%matches_dir%"+FOLDER_DELIM+"sfm_data.json", "-p", "%matches_dir%"+FOLDER_DELIM+"pairs.bin", "-o", "%matches_dir%"+FOLDER_DELIM+"matches.putative.bin", "-n", "AUTO"]],
            ["Filter matches",               # 4
             os.path.join(OPENMVG_BIN, "openMVG_main_GeometricFilter"),
             ["-i", "%matches_dir%"+FOLDER_DELIM+"sfm_data.json", "-m", "%matches_dir%"+FOLDER_DELIM+"matches.putative.bin", "-o", "%matches_dir%"+FOLDER_DELIM+"matches.f.bin"]],
            ["Incremental reconstruction",   # 5
             os.path.join(OPENMVG_BIN, "openMVG_main_SfM"),
             ["-i", "%matches_dir%"+FOLDER_DELIM+"sfm_data.json", "-m", "%matches_dir%", "-o", "%reconstruction_dir%", "-s", "INCREMENTAL"]],
            ["Global reconstruction",        # 6
             os.path.join(OPENMVG_BIN, "openMVG_main_SfM"),
             ["-i", "%matches_dir%"+FOLDER_DELIM+"sfm_data.json", "-m", "%matches_dir%", "-o", "%reconstruction_dir%", "-s", "GLOBAL", "-M", "%matches_dir%"+FOLDER_DELIM+"matches.e.bin"]],
            ["Colorize Structure",           # 7
             os.path.join(OPENMVG_BIN, "openMVG_main_ComputeSfM_DataColor"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"sfm_data.bin", "-o", "%reconstruction_dir%"+FOLDER_DELIM+"colorized.ply"]],
            ["Structure from Known Poses",   # 8
             os.path.join(OPENMVG_BIN, "openMVG_main_ComputeStructureFromKnownPoses"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"sfm_data.bin", "-m", "%matches_dir%", "-f", "%matches_dir%"+FOLDER_DELIM+"matches.f.bin", "-o", "%reconstruction_dir%"+FOLDER_DELIM+"robust.bin"]],
            ["Colorized robust triangulation",  # 9
             os.path.join(OPENMVG_BIN, "openMVG_main_ComputeSfM_DataColor"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"robust.bin", "-o", "%reconstruction_dir%"+FOLDER_DELIM+"robust_colorized.ply"]],
            ["Control Points Registration",  # 10
             os.path.join(OPENMVG_BIN, "ui_openMVG_control_points_registration"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"sfm_data.bin"]],
            ["Export to openMVS",            # 11
             os.path.join(OPENMVG_BIN, "openMVG_main_openMVG2openMVS"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"sfm_data.bin", "-o", "%mvs_dir%"+FOLDER_DELIM+"scene.mvs", "-d", "%mvs_dir%"+FOLDER_DELIM+"images"]],
            ["Feature Extractor",            # 12
             COLMAP_BIN,
             ["feature_extractor", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db", "--image_path", "%input_dir%"]],
            ["Exhaustive Matcher",           # 13
             COLMAP_BIN,
             ["exhaustive_matcher", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db"]],
            ["Mapper",                       # 14
             COLMAP_BIN,
             ["mapper", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db", "--image_path", "%input_dir%", "--output_path", "%reconstruction_dir%"]],
            ["Image Undistorter",            # 15
             COLMAP_BIN,
             ["image_undistorter", "--image_path", "%input_dir%", "--input_path", "%reconstruction_dir%"+FOLDER_DELIM+"0", "--output_path", "%reconstruction_dir%"+FOLDER_DELIM+"dense", "--output_type", "COLMAP"]],
            ["Export to openMVS",            # 16
             os.path.join(OPENMVS_BIN, "InterfaceCOLMAP"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"dense", "-o", "scene.mvs", "--image-folder", "%reconstruction_dir%"+FOLDER_DELIM+"dense"+FOLDER_DELIM+"images", "-w", "\"%mvs_dir%\""]],
            ["Densify point cloud",          # 17
             os.path.join(OPENMVS_BIN, "DensifyPointCloud"),
             ["scene.mvs", "--dense-config-file", "Densify.ini", "--resolution-level", "1", "--number-views", "8", "-w", "\"%mvs_dir%\""]],
            ["Reconstruct the mesh",         # 18
             os.path.join(OPENMVS_BIN, "ReconstructMesh"),
             ["scene_dense.mvs", "-p", "scene_dense.ply", "-w", "\"%mvs_dir%\""]],
            ["Refine the mesh",              # 19
             os.path.join(OPENMVS_BIN, "RefineMesh"),
             ["scene_dense.mvs", "-m", "scene_dense_mesh.ply", "-o", "scene_dense_mesh_refine.mvs", "--scales", "1", "--gradient-step", "25.05", "-w", "\"%mvs_dir%\""]],
            ["Texture the mesh",             # 20
             os.path.join(OPENMVS_BIN, "TextureMesh"),
             ["scene_dense.mvs", "-m", "scene_dense_mesh_refine.ply", "-o", "scene_dense_mesh_refine_texture.mvs", "--decimate", "0.5", "-w", "\"%mvs_dir%\""]],
            ["Estimate disparity-maps",      # 21
             os.path.join(OPENMVS_BIN, "DensifyPointCloud"),
             ["scene.mvs", "--dense-config-file", "Densify.ini", "--fusion-mode", "-1", "-w", "\"%mvs_dir%\""]],
            ["Fuse disparity-maps",          # 22
             os.path.join(OPENMVS_BIN, "DensifyPointCloud"),
             ["scene.mvs", "--dense-config-file", "Densify.ini", "--fusion-mode", "-2", "-w", "\"%mvs_dir%\""]]
            ]

    def __getitem__(self, indice):
        return AStep(*self.steps_data[indice])

    def length(self):
        return len(self.steps_data)

    def apply_conf(self, conf):
        """ replace each %var% per conf.var value in steps data """
        for s in self.steps_data:
            o2 = []
            for o in s[2]:
                co = o.replace("%input_dir%", conf.input_dir)
                co = co.replace("%output_dir%", conf.output_dir)
                co = co.replace("%matches_dir%", conf.matches_dir)
                co = co.replace("%reconstruction_dir%", conf.reconstruction_dir)
                co = co.replace("%mvs_dir%", conf.mvs_dir)
                co = co.replace("%camera_file_params%", conf.camera_file_params)
                o2.append(co)
            s[2] = o2

    def replace_opt(self, idx, str_exist, str_new):
        """ replace each existing str_exist with str_new per opt value in step idx data """
        s = self.steps_data[idx]
        o2 = []
        for o in s[2]:
            co = o.replace(str_exist, str_new)
            o2.append(co)
        s[2] = o2


CONF = ConfContainer()
STEPS = StepsStore()

# ARGS
PARSER = argparse.ArgumentParser(
    formatter_class=argparse.RawTextHelpFormatter,
    description="Photogrammetry reconstruction with these steps: \r\n" +
    "\r\n".join(("\t%i. %s\t %s" % (t, STEPS[t].info, STEPS[t].cmd) for t in range(STEPS.length())))
    )
PARSER.add_argument('input_dir',
                    help="the directory which contains the pictures set.")
PARSER.add_argument('output_dir',
                    help="the directory which will contain the resulting files.")
PARSER.add_argument('--steps',
                    type=int,
                    nargs="+",
                    help="steps to process")
PARSER.add_argument('--preset',
                    help="steps list preset in \r\n" +
                    " \r\n".join([k + " = " + str(PRESET[k]) for k in PRESET]) +
                    " \r\ndefault : " + PRESET_DEFAULT)

GROUP = PARSER.add_argument_group('Passthrough', description="Option to be passed to command lines (remove - in front of option names)\r\ne.g. --1 p ULTRA to use the ULTRA preset in openMVG_main_ComputeFeatures\r\nFor example, running the script as follows,\r\nMvgMvsPipeline.py input_dir output_dir --1 p HIGH n 8 --3 n ANNL2\r\nwhere --1 refer to openMVG_main_ComputeFeatures, p refers to\r\ndescriberPreset option which HIGH was chosen, and n refers to\r\nnumThreads which 8 was used. --3 refer to second step (openMVG_main_ComputeMatches),\r\nn refers to nearest_matching_method option which ANNL2 was chosen")
for n in range(STEPS.length()):
    GROUP.add_argument('--'+str(n), nargs='+')

PARSER.parse_args(namespace=CONF)  # store args in the ConfContainer


# FOLDERS

def mkdir_ine(dirname):
    """Create the folder if not presents"""
    if not os.path.exists(dirname):
        os.mkdir(dirname)


# Absolute path for input and output dirs
CONF.input_dir = os.path.abspath(CONF.input_dir)
CONF.output_dir = os.path.abspath(CONF.output_dir)

if not os.path.exists(CONF.input_dir):
    sys.exit("%s: path not found" % CONF.input_dir)

CONF.reconstruction_dir = os.path.join(CONF.output_dir, "sfm")
CONF.matches_dir = os.path.join(CONF.reconstruction_dir, "matches")
CONF.mvs_dir = os.path.join(CONF.output_dir, "mvs")
CONF.camera_file_params = os.path.join(CAMERA_SENSOR_DB_DIRECTORY, CAMERA_SENSOR_DB_FILE)

mkdir_ine(CONF.output_dir)
mkdir_ine(CONF.reconstruction_dir)
mkdir_ine(CONF.matches_dir)
mkdir_ine(CONF.mvs_dir)

# Update directories in steps commandlines
STEPS.apply_conf(CONF)

# PRESET
if CONF.steps and CONF.preset:
    sys.exit("Steps and preset arguments can't be set together.")
elif CONF.preset:
    try:
        CONF.steps = PRESET[CONF.preset]
    except KeyError:
        sys.exit("Unknown preset %s, choose %s" % (CONF.preset, ' or '.join([s for s in PRESET])))
elif not CONF.steps:
    CONF.steps = PRESET[PRESET_DEFAULT]

# WALK
print("# Using input dir:  %s" % CONF.input_dir)
print("#      output dir:  %s" % CONF.output_dir)
print("# Steps:  %s" % str(CONF.steps))

if 4 in CONF.steps:    # GeometricFilter
    if 6 in CONF.steps:  # GlobalReconstruction
        # Set the geometric_model of ComputeMatches to Essential
        STEPS.replace_opt(4, FOLDER_DELIM+"matches.f.bin", FOLDER_DELIM+"matches.e.bin")
        STEPS[4].opt.extend(["-g", "e"])

if 20 in CONF.steps:    # TextureMesh
    if 19 not in CONF.steps:  # RefineMesh
        # RefineMesh step is not run, use ReconstructMesh output
        STEPS.replace_opt(20, "scene_dense_mesh_refine.ply", "scene_dense_mesh.ply")
        STEPS.replace_opt(20, "scene_dense_mesh_refine_texture.mvs", "scene_dense_mesh_texture.mvs")

for cstep in CONF.steps:
    printout("#%i. %s" % (cstep, STEPS[cstep].info), effect=INVERSE)

    # Retrieve "passthrough" commandline options
    opt = getattr(CONF, str(cstep))
    if opt:
        # add - sign to short options and -- to long ones
        for o in range(0, len(opt), 2):
            if len(opt[o]) > 1:
                opt[o] = '-' + opt[o]
            opt[o] = '-' + opt[o]
    else:
        opt = []

    # Remove STEPS[cstep].opt options now defined in opt
    for anOpt in STEPS[cstep].opt:
        if anOpt in opt:
            idx = STEPS[cstep].opt.index(anOpt)
            if DEBUG:
                print('#\tRemove ' + str(anOpt) + ' from defaults options at id ' + str(idx))
            del STEPS[cstep].opt[idx:idx+2]

    # create a commandline for the current step
    cmdline = [STEPS[cstep].cmd] + STEPS[cstep].opt + opt
    print('Cmd: ' + ' '.join(cmdline))

    if not DEBUG:
        # Launch the current step
        try:
            pStep = subprocess.Popen(cmdline)
            pStep.wait()
            if pStep.returncode != 0:
                break
        except KeyboardInterrupt:
            sys.exit('\r\nProcess canceled by user, all files remains')
    else:
        print('\t'.join(cmdline))

printout("# Pipeline end #", effect=INVERSE)
