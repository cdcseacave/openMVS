#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
This script is for an easy use of OpenMVS, with optional OpenMVG or COLMAP frontends

By default the script runs the fully native OpenMVS pipeline (`CreateStructure`
for sparse SfM, then dense / mesh / refine / texture). The OpenMVG and COLMAP
binaries are only consulted when a preset that needs them is selected — they
do not have to be installed for the default `NATIVE` preset.

Example usage:
  python3 MvgMvs_Pipeline.py [-h]
    input_dir output_dir
    [--steps STEPS [STEPS ...]] [--preset PRESET]
    [--0 0 [0 ...]] [--1 1 [1 ...]] [--2 2 [2 ...]]
    [--3 3 [3 ...]] [--4 4 [4 ...]] [--5 5 [5 ...]]
    [--6 6 [6 ...]] [--7 7 [7 ...]] [--8 8 [8 ...]]
    [--9 9 [9 ...]] [--10 10 [10 ...]] [--11 11 [11 ...]]
    [--12 12 [12 ...]] [--13 13 [13 ...]] [--14 14 [14 ...]]
    [--15 15 [15 ...]] [--16 16 [16 ...]] [--17 17 [17 ...]]
    [--18 18 [18 ...]] [--19 19 [19 ...]] [--20 20 [20 ...]]
    [--21 21 [21 ...]] [--22 22 [22 ...]] [--23 23 [23 ...]]
    [--24 24 [24 ...]] [--25 25 [25 ...]]

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
    15. Model Aligner                  colmap
    16. Image Undistorter              colmap
    17. Export to openMVS              InterfaceCOLMAP
    18. Densify point-cloud            DensifyPointCloud
    19. Reconstruct the mesh           ReconstructMesh
    20. Refine the mesh                RefineMesh
    21. Texture the mesh               TextureMesh
    22. Estimate disparity-maps        DensifyPointCloud
    23. Fuse disparity-maps            DensifyPointCloud
    24. Sparse reconstruction (native) CreateStructure
    25. Extract video keyframes        ExtractKeyframes (auto-inserted when input_dir is a video file)

Positional arguments:
  input_dir                 the directory which contains the pictures set.
  output_dir                the directory which will contain the resulting files.

Optional arguments:
  -h, --help                show this help message and exit
  --steps STEPS [STEPS ...] steps to process
  --preset PRESET           steps list preset in
                            NATIVE = [24, 18, 19, 20, 21]
                            SEQUENTIAL = [0, 1, 2, 3, 4, 5, 11, 18, 19, 20, 21]
                            GLOBAL = [0, 1, 2, 3, 4, 6, 11, 18, 19, 20, 21]
                            MVG_SEQ = [0, 1, 2, 3, 4, 5, 7, 8, 9, 11]
                            MVG_GLOBAL = [0, 1, 2, 3, 4, 6, 7, 8, 9, 11]
                            COLMAP_MVS = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
                            COLMAP = [12, 13, 14, 15, 16, 17]
                            MVS = [18, 19, 20, 21]
                            MVS_SGM = [22, 23]
                            default : NATIVE

Passthrough:
  Option to be passed to command lines (remove - in front of option names)
  e.g. --1 p ULTRA to use the ULTRA preset in openMVG_main_ComputeFeatures
  For example, running the script
  [MvgMvsPipeline.py input_dir output_dir --steps 0 1 2 3 4 5 11 18 19 21 --1 p HIGH n 8 --3 n HNSWL2]
  [--steps 0 1 2 3 4 5 11 18 19 21] runs only the desired steps
  [--1 p HIGH n 8] where --1 refer to openMVG_main_ComputeFeatures,
  p refers to describerPreset option and set to HIGH, and n refers
  to numThreads and set to 8. The second step (Compute matches),
  [--3 n HNSWL2] where --3 refer to openMVG_main_ComputeMatches,
  n refers to nearest_matching_method option and set to HNSWL2

  COLMAP with ALIKED + LightGlue (deep features + learned matcher), both on CPU and on GPU:
  [MvgMvsPipeline.py images_dir out_dir --preset COLMAP_MVS
   --12 FeatureExtraction.type ALIKED_N16ROT AlikedExtraction.max_num_features 4096
   --13 FeatureMatching.type ALIKED_LIGHTGLUE]
  --12 forwards to colmap feature_extractor; --13 to exhaustive_matcher.
  Append  FeatureExtraction.use_gpu 0  /  FeatureMatching.use_gpu 0  to the
  passthroughs to force the CPU ONNX provider (works without any CUDA/cuDNN install).
  GPU requires: a colmap-bundled onnxruntime built against the host's CUDA major version.
  Available extractor types: SIFT, ALIKED_N16ROT, ALIKED_N32.
  Available matcher types: SIFT_BRUTEFORCE, SIFT_LIGHTGLUE,
  ALIKED_BRUTEFORCE, ALIKED_LIGHTGLUE. Requires colmap >= 3.14.

Created by @FlachyJoe
"""

import os
import sqlite3
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
        return directory in which afile is, empty string if not found. Look in PATH
    """
    if sys.platform.startswith('win'):
        cmd = "where"
    else:
        cmd = "which"
    try:
        ret = subprocess.run([cmd, afile], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True)
        # `where` on Windows can return multiple matches (one per line) when the
        # same executable is found in several PATH entries — keep only the first.
        first = ret.stdout.decode().splitlines()[0].strip()
        return os.path.split(first)[0]
    except (subprocess.CalledProcessError, IndexError):
        return ''


def find(afile):
    """
        As whereis look only for executable on linux, this find look for all file type
    """
    for d in os.environ['PATH'].split(PATH_DELIM):
        if os.path.isfile(os.path.join(d, afile)):
            return d
    return None


def count_db_gps_priors(db_path):
    """Count WGS84 pose-priors in a COLMAP database (returns 0 if the database
    or the pose_priors table is missing). WGS84 == coordinate_system 0 per
    PosePrior::CoordinateSystem in COLMAP's geometry/pose_prior.h."""
    if not os.path.isfile(db_path):
        return 0
    try:
        con = sqlite3.connect(db_path)
        try:
            cur = con.execute(
                "SELECT COUNT(*) FROM pose_priors WHERE coordinate_system = 0")
            return cur.fetchone()[0]
        finally:
            con.close()
    except sqlite3.DatabaseError:
        return 0


# Try to find openMVG, COLMAP, and openMVS binaries in PATH.
# Missing entries are deferred — we only prompt for binaries actually used by
# the resolved step list (see ensure_binaries() below). This keeps the default
# native-SfM run usable without OpenMVG or COLMAP installed.
OPENMVG_BIN = whereis("openMVG_main_SfMInit_ImageListing")
COLMAP_BIN = whereis("colmap")
OPENMVS_BIN = whereis("ReconstructMesh")

CAMERA_SENSOR_DB_FILE = "sensor_width_camera_database.txt"
CAMERA_SENSOR_DB_DIRECTORY = find(CAMERA_SENSOR_DB_FILE) or ''

PRESET = {'NATIVE': [24, 18, 19, 20, 21],
          'SEQUENTIAL': [0, 1, 2, 3, 4, 5, 11, 18, 19, 20, 21],
          'GLOBAL': [0, 1, 2, 3, 4, 6, 11, 18, 19, 20, 21],
          'MVG_SEQ': [0, 1, 2, 3, 4, 5, 7, 8, 9, 11],
          'MVG_GLOBAL': [0, 1, 2, 3, 4, 6, 7, 8, 9, 11],
          'COLMAP_MVS': [12, 13, 14, 15, 16, 17, 18, 19, 20, 21],
          'COLMAP': [12, 13, 14, 15, 16, 17],
          'MVS': [18, 19, 20, 21],
          'MVS_SGM': [22, 23]}

PRESET_DEFAULT = 'NATIVE'

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
        sys.stdout.write(seq+'\n')
    else:
        sys.stdout.write(text+'\n')


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
             ["feature_extractor", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db", "--image_path", "%input_dir%", "--ImageReader.camera_model=OPENCV"]],
            ["Exhaustive Matcher",           # 13
             COLMAP_BIN,
             ["exhaustive_matcher", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db"]],
            ["Mapper",                       # 14
             COLMAP_BIN,
             ["mapper", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db", "--image_path", "%input_dir%", "--output_path", "%reconstruction_dir%"]],
            ["Model Aligner",                # 15
             COLMAP_BIN,
             ["model_aligner", "--input_path", "%reconstruction_dir%"+FOLDER_DELIM+"0", "--database_path", "%matches_dir%"+FOLDER_DELIM+"database.db", "--output_path", "%reconstruction_dir%"+FOLDER_DELIM+"0", "--ref_is_gps=1", "--alignment_max_error=2.0", "--alignment_type=enu", "--transform_path", "%reconstruction_dir%"+FOLDER_DELIM+"transform.txt"]],
            ["Image Undistorter",            # 16
             COLMAP_BIN,
             ["image_undistorter", "--image_path", "%input_dir%", "--input_path", "%reconstruction_dir%"+FOLDER_DELIM+"0", "--output_path", "%reconstruction_dir%"+FOLDER_DELIM+"dense", "--output_type", "COLMAP"]],
            ["Export to openMVS",            # 17
             os.path.join(OPENMVS_BIN, "InterfaceCOLMAP"),
             ["-i", "%reconstruction_dir%"+FOLDER_DELIM+"dense", "-o", "scene.mvs", "--image-folder", "%reconstruction_dir%"+FOLDER_DELIM+"dense"+FOLDER_DELIM+"images", "-w", "\"%mvs_dir%\""]],
            ["Densify point cloud",          # 18
             os.path.join(OPENMVS_BIN, "DensifyPointCloud"),
             ["scene.mvs", "--dense-config-file", "Densify.ini", "--resolution-level", "1", "--number-views", "8", "-w", "\"%mvs_dir%\""]],
            ["Reconstruct the mesh",         # 19
             os.path.join(OPENMVS_BIN, "ReconstructMesh"),
             ["scene_dense.mvs", "-p", "scene_dense.ply", "-w", "\"%mvs_dir%\""]],
            ["Refine the mesh",              # 20
             os.path.join(OPENMVS_BIN, "RefineMesh"),
             ["scene_dense.mvs", "-m", "scene_dense_mesh.ply", "-o", "scene_dense_mesh_refine.mvs", "--scales", "1", "--gradient-step", "25.05", "-w", "\"%mvs_dir%\""]],
            ["Texture the mesh",             # 21
             os.path.join(OPENMVS_BIN, "TextureMesh"),
             ["scene_dense.mvs", "-m", "scene_dense_mesh_refine.ply", "--decimate", "0.5", "-w", "\"%mvs_dir%\""]],
            ["Estimate disparity-maps",      # 22
             os.path.join(OPENMVS_BIN, "DensifyPointCloud"),
             ["scene.mvs", "--dense-config-file", "Densify.ini", "--fusion-mode", "-1", "-w", "\"%mvs_dir%\""]],
            ["Fuse disparity-maps",          # 23
             os.path.join(OPENMVS_BIN, "DensifyPointCloud"),
             ["scene.mvs", "--dense-config-file", "Densify.ini", "--fusion-mode", "-2", "-w", "\"%mvs_dir%\""]],
            ["Sparse reconstruction (native SfM)",  # 24
             os.path.join(OPENMVS_BIN, "CreateStructure"),
             ["-s", "%input_dir%", "-o", "scene.sfm", "--export-mvs", "scene.mvs", "--extract-colors", "1", "-w", "\"%mvs_dir%\""]],
            ["Extract video keyframes",      # 25
             os.path.join(OPENMVS_BIN, "ExtractKeyframes"),
             ["-i", "%input_dir%", "-o", "scene_keyframes.sfm", "-d", "keyframes", "-w", "\"%mvs_dir%\""]]
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


# Step number -> toolchain whose bin folder must be prompted for if missing.
# Used by ensure_binaries() so the default NATIVE preset does not have to ask
# for OpenMVG or COLMAP folders when only OpenMVS steps will run.
OPENMVG_STEPS = set(range(0, 12))   # 0..11
COLMAP_STEPS = set(range(12, 17))   # 12..16
OPENMVS_STEPS = set(range(17, 26))  # 17..25


def _peek_steps_list():
    """Return the step list argv requested, without committing to the full
    argparse spec — the full parser's help text references STEPS, which cannot
    be built until the bin folders are resolved."""
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--steps', type=int, nargs="+")
    p.add_argument('--preset')
    pre, _ = p.parse_known_args()
    if pre.steps and pre.preset:
        sys.exit("Steps and preset arguments can't be set together.")
    if pre.preset:
        if pre.preset not in PRESET:
            sys.exit("Unknown preset %s, choose %s" % (pre.preset, ' or '.join(PRESET)))
        return PRESET[pre.preset]
    if pre.steps:
        return pre.steps
    return PRESET[PRESET_DEFAULT]


def ensure_binaries(steps_to_run):
    """Prompt for OpenMVG / COLMAP / openMVS / sensor-DB folders only for the
    toolchains that the resolved step list will actually invoke. Run before
    StepsStore() is constructed so os.path.join(BIN, "name") sees real paths."""
    global OPENMVG_BIN, COLMAP_BIN, OPENMVS_BIN, CAMERA_SENSOR_DB_DIRECTORY
    steps_set = set(steps_to_run)
    if steps_set & OPENMVG_STEPS and not OPENMVG_BIN:
        OPENMVG_BIN = input("openMVG binary folder?\n")
    if steps_set & COLMAP_STEPS and not COLMAP_BIN:
        COLMAP_BIN = input("COLMAP binary folder?\n")
    if steps_set & OPENMVS_STEPS and not OPENMVS_BIN:
        OPENMVS_BIN = input("openMVS binary folder?\n")
    if 0 in steps_set and not CAMERA_SENSOR_DB_DIRECTORY:
        CAMERA_SENSOR_DB_DIRECTORY = input(
            "openMVG camera database (%s) folder?\n" % CAMERA_SENSOR_DB_FILE)
    # Append the colmap executable to the directory once, as in the original.
    # Harmless when COLMAP_BIN is empty: os.path.join('', 'colmap') == 'colmap',
    # which is only baked into steps_data entries that won't be invoked.
    COLMAP_BIN = os.path.join(COLMAP_BIN, "colmap")
    if sys.platform.startswith('win'):
        COLMAP_BIN += ".bat"


# Skip binary prompting when the user just wants -h/--help; argparse will
# still build STEPS below with whatever BIN globals are currently set (which
# is fine for help text — only command execution needs real paths).
if not ({'-h', '--help'} & set(sys.argv[1:])):
    ensure_binaries(_peek_steps_list())


CONF = ConfContainer()
STEPS = StepsStore()

# ARGS
PARSER = argparse.ArgumentParser(
    formatter_class=argparse.RawTextHelpFormatter,
    description="Photogrammetry reconstruction with these steps:\n" +
    "\n".join(("\t%i. %s\t %s" % (t, STEPS[t].info, STEPS[t].cmd) for t in range(STEPS.length())))
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
                    help="steps list preset in\n" +
                    " \n".join([k + " = " + str(PRESET[k]) for k in PRESET]) +
                    " \ndefault : " + PRESET_DEFAULT)

GROUP = PARSER.add_argument_group('Passthrough', description="Option to be passed to command lines (remove - in front of option names)\nex. --1 p ULTRA to use the ULTRA preset in openMVG_main_ComputeFeatures\nFor example, running the script as follows,\nMvgMvsPipeline.py input_dir output_dir --1 p HIGH n 8 --3 n ANNL2\nwhere --1 refer to openMVG_main_ComputeFeatures, p refers to\ndescriberPreset option which HIGH was chosen, and n refers to\nnumThreads which 8 was used. --3 refer to second step (openMVG_main_ComputeMatches),\nn refers to nearest_matching_method option which ANNL2 was chosen\n\nCOLMAP with ALIKED + LightGlue (requires colmap >= 3.14):\nMvgMvsPipeline.py images_dir out_dir --preset COLMAP_MVS \\\n  --12 FeatureExtraction.type ALIKED_N16ROT AlikedExtraction.max_num_features 4096 \\\n  --13 FeatureMatching.type ALIKED_LIGHTGLUE\nAppend FeatureExtraction.use_gpu 0 / FeatureMatching.use_gpu 0 to fall back\nto the CPU ONNX provider if onnxruntime can't load its CUDA provider.\nExtractor types: SIFT, ALIKED_N16ROT, ALIKED_N32.\nMatcher types: SIFT_BRUTEFORCE, SIFT_LIGHTGLUE, ALIKED_BRUTEFORCE, ALIKED_LIGHTGLUE.")
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

# Resolve the effective step list up-front so the folder layout can be chosen
# based on whether OpenMVG/COLMAP staging directories are actually needed.
if CONF.steps and CONF.preset:
    sys.exit("Steps and preset arguments can't be set together.")
elif CONF.preset:
    try:
        CONF.steps = PRESET[CONF.preset]
    except KeyError:
        sys.exit("Unknown preset %s, choose %s" % (CONF.preset, ' or '.join([s for s in PRESET])))
elif not CONF.steps:
    CONF.steps = PRESET[PRESET_DEFAULT]

# Pure-OpenMVS runs write straight into output_dir; the sfm/ + matches/ + mvs/
# subfolders only stage per-toolchain intermediates for OpenMVG/COLMAP presets.
NATIVE_ONLY = not (set(CONF.steps) & (OPENMVG_STEPS | COLMAP_STEPS))
CONF.mvs_dir = CONF.output_dir if NATIVE_ONLY else os.path.join(CONF.output_dir, "mvs")
CONF.reconstruction_dir = CONF.output_dir if NATIVE_ONLY else os.path.join(CONF.output_dir, "sfm")
CONF.matches_dir = CONF.output_dir if NATIVE_ONLY else os.path.join(CONF.reconstruction_dir, "matches")
CONF.camera_file_params = os.path.join(CAMERA_SENSOR_DB_DIRECTORY, CAMERA_SENSOR_DB_FILE)

mkdir_ine(CONF.output_dir)
if not NATIVE_ONLY:
    mkdir_ine(CONF.reconstruction_dir)
    mkdir_ine(CONF.matches_dir)
    mkdir_ine(CONF.mvs_dir)

# Update directories in steps commandlines
STEPS.apply_conf(CONF)


# Video container extensions auto-routed through ExtractKeyframes (step 25)
# when the input is a single file rather than a folder of images.
VIDEO_EXTS = {'.mp4', '.mov', '.mkv', '.avi', '.webm', '.m4v', '.mpg', '.mpeg', '.wmv', '.3gp', '.ts'}


def is_video_input(path):
    return os.path.isfile(path) and os.path.splitext(path)[1].lower() in VIDEO_EXTS


# Auto-promote a video input to ExtractKeyframes → CreateStructure (only the
# native SfM step understands a .sfm produced by ExtractKeyframes; for OpenMVG
# / COLMAP frontends we leave the steps untouched and let them fail clearly).
if is_video_input(CONF.input_dir) and 24 in CONF.steps and 25 not in CONF.steps:
    print("# Video input detected — auto-inserting ExtractKeyframes (step 25) before CreateStructure")
    CONF.steps = list(CONF.steps)
    CONF.steps.insert(CONF.steps.index(24), 25)
    STEPS.replace_opt(24, CONF.input_dir, "scene_keyframes.sfm")

# WALK
print("# Using input dir:  %s" % CONF.input_dir)
print("#      output dir:  %s" % CONF.output_dir)
print("# Steps:  %s" % str(CONF.steps))

if 4 in CONF.steps:    # GeometricFilter
    if 6 in CONF.steps:  # GlobalReconstruction
        # Set the geometric_model of ComputeMatches to Essential
        STEPS.replace_opt(4, FOLDER_DELIM+"matches.f.bin", FOLDER_DELIM+"matches.e.bin")
        STEPS[4].opt.extend(["-g", "e"])

if 21 in CONF.steps:    # TextureMesh
    if 20 not in CONF.steps:  # RefineMesh
        # RefineMesh step is not run, use ReconstructMesh output
        STEPS.replace_opt(21, "scene_dense_mesh_refine.ply", "scene_dense_mesh.ply")
        STEPS.replace_opt(21, "scene_dense_mesh_refine_texture.mvs", "scene_dense_mesh_texture.mvs")

for cstep in CONF.steps:
    printout("#%i. %s" % (cstep, STEPS[cstep].info), effect=INVERSE)

    # Step 15 (COLMAP model_aligner): only request GPS/ENU alignment when the
    # database actually has enough WGS84 pose priors, otherwise drop straight to
    # principal-plane alignment. Skips the buggy GPS code path on no-GPS scenes
    # (fixed upstream by colmap 687f8e5e, but older releases still crash).
    if cstep == 15 and "--ref_is_gps=1" in STEPS[15].opt:
        db_path = os.path.join(CONF.matches_dir, "database.db")
        n_gps = count_db_gps_priors(db_path)
        if n_gps < 3:
            printout("# No GPS pose priors in database (%d WGS84 rows) — using plane alignment" % n_gps, effect=INVERSE)
            STEPS.replace_opt(15, "--ref_is_gps=1", "--ref_is_gps=0")
            STEPS.replace_opt(15, "--alignment_type=enu", "--alignment_type=plane")
            # Drop --database_path: colmap's RunModelAligner reads pose priors
            # from it even when alignment_type=plane, hitting the buggy path.
            step15_opts = STEPS.steps_data[15][2]
            if "--database_path" in step15_opts:
                idx = step15_opts.index("--database_path")
                del step15_opts[idx:idx+2]
        else:
            printout("# %d GPS pose priors found — aligning scene to ENU/WGS84" % n_gps, effect=INVERSE)

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
            if subprocess.run(cmdline, check=True).returncode != 0:
                break
        except subprocess.CalledProcessError:
            # check if this COLMAP model-aligner step, retry using plane alignment instead of GPS
            if cstep == 15 and "--ref_is_gps=1" in STEPS[cstep].opt:
                printout("# Retry COLMAP model-aligner step using plane alignment instead of GPS", effect=INVERSE)
                STEPS.replace_opt(15, "--ref_is_gps=1", "--ref_is_gps=0")
                STEPS.replace_opt(15, "--alignment_type=enu", "--alignment_type=plane")
                cmdline = [STEPS[cstep].cmd] + STEPS[cstep].opt + opt
                print('Cmd: ' + ' '.join(cmdline))
                try:
                    if subprocess.run(cmdline, check=True).returncode != 0:
                        break
                except subprocess.CalledProcessError:
                    sys.exit('\nProcess failed at step %i (model_aligner GPS and plane both failed)' % cstep)
            else:
                sys.exit('\nProcess failed at step %i' % cstep)
        except KeyboardInterrupt:
            sys.exit('\nProcess canceled by user at step %i, all files remains' % cstep)
    else:
        print('\t'.join(cmdline))

printout("# Pipeline end #", effect=INVERSE)
