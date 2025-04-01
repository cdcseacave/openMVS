#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Import depth-maps corresponding to the given scene images, stored as plain EXR depth images.
Each scene image should have a corresponding depth-map with the same name, but with a '.depth.exr' extension.

Install:
  pip install opencv-python-headless numpy tqdm argparse Imath OpenEXR scikit-learn

Example usage:
  python3 ImportDMAPs.py [-h] --scene MVS_SCENE_FILE --input DEPTH_DIR [--ext EXT] [--output OUTPUT_DIR]
"""

from argparse import ArgumentParser
from MvsUtils import loadMVSInterface, scale_K, sample_depth_map, saveDMAP
from tqdm import tqdm
import numpy as np
import os


def load_depth_npy(exr_path):
  """
  Load depth data from an NPY file.
  Args:
    exr_path (str): Path to the NPY file.
  Returns:
    numpy.ndarray: Depth data as a NumPy array.
  """
  try:
    depth_map = np.load(exr_path)
    depth_map = np.nan_to_num(depth_map, nan=0.0, posinf=0.0, neginf=0.0)
    return depth_map
  except Exception as e:
    print(f"Error loading NPY file {exr_path}: {e}")
    return None


def load_depth_exr(exr_path):
  """
  Load depth data from an EXR file.
  Note: This will use the first channel in the EXR file.
  Args:
    exr_path (str): Path to the EXR file.
  Returns:
    numpy.ndarray: Depth data as a NumPy array.
  """
  import Imath
  import OpenEXR

  try:
    exr_file = OpenEXR.InputFile(exr_path)
    header = exr_file.header()
    channels = header['channels'].keys()
    dw = header["dataWindow"]
    size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    pt = Imath.PixelType(Imath.PixelType.FLOAT)
    depth_str = exr_file.channel(list(channels)[0], pt)
    depth_map = np.frombuffer(depth_str, dtype=np.float32).reshape((size[1], size[0]))
    depth_map = np.nan_to_num(depth_map, nan=0.0, posinf=0.0, neginf=0.0)
    return depth_map
  except Exception as e:
    print(f"Error loading EXR file {exr_path}: {e}")
    return None


def scale_depth_map(scene, image_idx, depth_map, verbose=False):
  """
  Estimate the scale and shift of the depth map based on the scene sparse point cloud,
  ussing RANSAC to find the best fit:
    depth_map_scaled = scale * depth_map + shift
  Args:
    scene (dict): The MVS scene data.
    image_idx (int): The index of the image in the scene.
    depth_map (numpy.ndarray): The depth map to be scaled corresponding to the image.
    verbose (bool): If True, print debug information.
  Returns:
    tuple: Scale and shift values.
  """
  from sklearn.linear_model import RANSACRegressor

  # Collect 3D points and corresponding depth values
  image = scene["images"][image_idx]
  image_width  = scene["platforms"][image["platform_id"]]["cameras"][image["camera_id"]]["width"]
  image_height = scene["platforms"][image["platform_id"]]["cameras"][image["camera_id"]]["height"]
  K = np.array(scene["platforms"][image["platform_id"]]["cameras"][image["camera_id"]]["K"])
  R = np.array(scene["platforms"][image["platform_id"]]["poses"][image["pose_id"]]["R"])
  C = np.array(scene["platforms"][image["platform_id"]]["poses"][image["pose_id"]]["C"])
  K = scale_K(K, depth_map.shape[1] / image_width, depth_map.shape[0] / image_height)
  depths_sfm = []
  depths_dmap = []
  mean_depth = 0
  for vertex in scene['vertices']:
    for view in vertex['views']:
      if view['image_id'] == image_idx:
        # Project the 3D point to the image plane
        # and get the corresponding depth value
        Xcam = R @ (vertex['X'] - C)
        depth_sfm = float(Xcam[2])
        if depth_sfm <= 0:
          break
        x = K @ Xcam
        x = np.array([x[0]/x[2], x[1]/x[2]])
        depth_dmap = sample_depth_map(depth_map, x)
        if depth_dmap <= 0:
          break
        depths_sfm.append(depth_sfm)
        depths_dmap.append(depth_dmap)
        mean_depth += depth_sfm
        break
  if len(depths_sfm) < 2:
    return 1.0, 0.0
  mean_depth /= len(depths_sfm)
  depths_sfm = np.array(depths_sfm).reshape(-1, 1)
  depths_dmap = np.array(depths_dmap).reshape(-1, 1)
  
  # Define the estimator, with all the functions required by RANSAC
  class Estimator:
    def __init__(self, scale=1.0, shift=0.0):
      self.scale = scale
      self.shift = shift

    def fit(self, X, y):
      # Solve for scale and shift
      assert len(X) >= 2, "At least two samples are required for RANSAC"    
      if X[1][0] - X[0][0] == 0:
        return
      scale = (y[1][0] - y[0][0]) / (X[1][0] - X[0][0])
      if scale <= 0:
        return
      self.scale = scale
      self.shift = y[0][0] - scale * X[0][0]

    def predict(self, X):
      return self.scale * X + self.shift    

    def score(self, X, y):
      return np.mean(np.abs(self.predict(X) - y))
    
    def get_params(self, deep=True):
      return {'scale': self.scale, 'shift': self.shift}
    
    def set_params(self, **params):
      if 'scale' in params:
        self.scale = params['scale']
      if 'shift' in params:
        self.shift = params['shift']

  # RANSAC model
  inlier_threshold = mean_depth * 0.03
  ransac = RANSACRegressor(
    estimator=Estimator(),
    residual_threshold=inlier_threshold,
    min_samples=2,
    max_trials=1000,
    loss='absolute_error',
    stop_probability=0.99999,
  )
  
  # Fit RANSAC model
  ransac.fit(depths_dmap, depths_sfm)
  score = ransac.score(depths_dmap[ransac.inlier_mask_], depths_sfm[ransac.inlier_mask_])

  # Print the number of inliers
  if verbose:
    num_inliers = np.sum(ransac.inlier_mask_)
    print(f"RANSAC stats: {ransac.n_trials_} iterations, {score:.4f} score, {num_inliers} / {len(depths_dmap)} inliers")
  
  # Get the scale and shift from the model
  scale = ransac.estimator_.scale
  shift = ransac.estimator_.shift
  assert scale > 0, "Scale must be positive after RANSAC"

  # Non-linear optimization to refine the scale and shift using Huber robust loss
  from scipy.optimize import minimize
  def objective(params, X, y, delta):
    scale, shift = params
    residuals = np.abs(scale * X + shift - y)
    loss = np.where(residuals <= delta, 0.5 * residuals**2, delta * (residuals - 0.5 * delta))
    return np.sum(loss)
  result = minimize(objective, [scale, shift], args=(depths_dmap[ransac.inlier_mask_], depths_sfm[ransac.inlier_mask_], inlier_threshold/2), method='L-BFGS-B')
  if result.success:
    if verbose:
      print(f"Optimization stats: {result.nit} iterations, {result.fun/num_inliers:.4f} score, {result.message}")
    scale, shift = result.x
    assert scale > 0, "Scale must be positive after optimization"
  if verbose:
    print(f"Estimated scale: {scale:.4f}, shift: {shift:.4f}")
  return scale, shift


def import_dmaps(scene_file, input_dir, ext, output_file, rescale=True, verbose=False):
  """
  Import depth maps from EXR files and save them as DMAP files.
  Args:
    scene_file (str): Path to the MVS scene file.
    input_dir (str): Directory containing the depth files.
    ext (str): Extension of the depth files to load (e.g., '.npy' or '.depth.exr').
    output_file (str): Directory to save the DMAP files.
    verbose (bool): If True, print debug information.
  """
  # Load the MVS scene
  scene = loadMVSInterface(scene_file)
  if verbose:
    print(f"Scene {scene_file} loaded: {len(scene['images'])} images")

  os.makedirs(output_file, exist_ok=True)

  for idx, image in tqdm(enumerate(scene["images"]), desc="Importing depth-maps", total=len(scene["images"])):
    image_name_ext = os.path.basename(image["name"])
    image_name = os.path.splitext(image_name_ext)[0]
    depth_file_path = os.path.join(input_dir, image_name + ext)
    if not os.path.exists(depth_file_path):
      print(f"Warning: Depth file not found for {depth_file_path}")
      continue

    # Load the depth map from the corresponding file
    if depth_file_path.endswith(".npy"):
      depth_map = load_depth_npy(depth_file_path)
    else:
      depth_map = load_depth_exr(depth_file_path)
    if depth_map is None:
      print(f"Warning: Could not load depth map for {image_name}")
      continue

    if rescale:
      # Scale and shift the depth map
      scale, shift = scale_depth_map(scene, idx, depth_map, verbose)
      depth_map[depth_map != 0] = scale * depth_map[depth_map != 0] + shift

    # Create DMAP data
    dmap_data = {
      "has_normal": False,
      "has_conf": False,
      "has_views": False,
      "image_width": scene["platforms"][image["platform_id"]]["cameras"][image["camera_id"]]["width"],
      "image_height": scene["platforms"][image["platform_id"]]["cameras"][image["camera_id"]]["height"],
      "depth_width": depth_map.shape[1],
      "depth_height": depth_map.shape[0],
      "depth_min": np.min(depth_map[depth_map > 0]) if np.any(depth_map > 0) else 0,
      "depth_max": np.max(depth_map),
      "file_name": image["name"],
      "reference_view_id": image["id"],
      "neighbor_view_ids": [],
      "K": scene["platforms"][image["platform_id"]]["cameras"][image["camera_id"]]["K"],
      "R": scene["platforms"][image["platform_id"]]["poses"][image["pose_id"]]["R"],
      "C": scene["platforms"][image["platform_id"]]["poses"][image["pose_id"]]["C"],
      "depth_map": depth_map,
    }

    # Save DMAP
    dmap_output_path = os.path.join(output_file, f"depth{image['id']:04d}.dmap")
    saveDMAP(dmap_data, dmap_output_path)


if __name__ == "__main__":
  parser = ArgumentParser()
  parser.add_argument(
    "-s", "--scene", type=str, required=True, help="File path to the MVS scene"
  )
  parser.add_argument(
    "-i", "--input", type=str, required=True, help="Path to the plain NPY or EXR depth images"
  )
  parser.add_argument(
    "-e", "--ext", type=str, default=".npy", help="Extension of the depth files to load (e.g., '.npy' or '.depth.exr')",
  )
  parser.add_argument(
    "-o", "--output", type=str, default=".", help="Path where to store the DMAP files",
  )
  parser.add_argument(
    "-r", "--rescale", action="store_true", help="Rescale the depth maps using SfM point cloud",
  )
  parser.add_argument(
    "-v", "--verbose", action="store_true", help="Enable verbose output"
  )
  args = parser.parse_args()
  
  import_dmaps(args.scene, args.input, args.ext, args.output, args.rescale, args.verbose)
