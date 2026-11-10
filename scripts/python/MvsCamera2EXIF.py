#!/usr/bin/python3
# -*- encoding: utf-8 -*-
'''
Reads camera intrinsics from an MVS scene and adds EXIF information to image files.
This script calculates the 35mm equivalent focal length and adds camera model info
to differentiate images taken by different cameras/platforms.

The script processes an MVS scene file and adds or updates EXIF metadata for each
referenced image with:
- 35mm equivalent focal length calculated from camera intrinsics
- Camera model name derived from platform and camera names in the MVS scene
- Camera make set to "OpenMVS"

This is useful for:
- Adding missing EXIF data to images processed through OpenMVS pipeline
- Differentiating images from different cameras/platforms
- Providing focal length information for photo viewers and other software

Install dependencies:
  pip install numpy pillow piexif

Example usage:
  python MvsCamera2EXIF.py -i scene.mvs -p /path/to/images
  python MvsCamera2EXIF.py -i scene.mvs -p /path/to/images --dry-run

usage: MvsCamera2EXIF.py [-h] [--input INPUT] [--images-path IMAGES_PATH] [--dry-run]
'''

from argparse import ArgumentParser
from MvsUtils import loadMVSInterface, CAMERA_MODEL_SPHERICAL
import os

try:
  import piexif
  from PIL import Image
  DEPENDENCIES_AVAILABLE = True
except ImportError as e:
  print(f"Warning: Missing dependencies: {e}")
  print("Please install with: pip install pillow piexif")
  DEPENDENCIES_AVAILABLE = False


def calculate_35mm_focal_length(fx, fy, width, height):
  """
  Calculate the 35mm equivalent focal length from camera intrinsics.
  
  Args:
    fx, fy: Focal length in pixels
    width, height: Image dimensions in pixels
  
  Returns:
    35mm equivalent focal length in mm
  """
  # Use the larger focal length value
  f_pixels = max(fx, fy)

  # Calculate 35mm equivalent focal length
  # f_35mm = f_pixels * (35mm_sensor_width / actual_sensor_width) * (actual_sensor_width / image_width)
  # Simplified: f_35mm = f_pixels * 36.0 / image_width
  f_35mm = f_pixels * 36.0 / width
  
  return round(f_35mm, 1)


def get_camera_model_name(platform_name, camera_name):
  """
  Generate a camera model name from platform and camera information.
  """
  if platform_name and camera_name:
    if platform_name.lower() in camera_name.lower():
      return camera_name
    else:
      return f"{platform_name}_{camera_name}"
  elif camera_name:
    return camera_name
  elif platform_name:
    return platform_name
  else:
    return "Unknown_Camera"


def add_exif_to_image(image_path, focal_length_35mm, camera_model, dry_run=False):
  """
  Add or update EXIF data in an image file.
  
  Args:
    image_path: Path to the image file
    focal_length_35mm: 35mm equivalent focal length
    camera_model: Camera model name
    dry_run: If True, only print what would be done
  """
  if not DEPENDENCIES_AVAILABLE:
    print(f"Error: Cannot modify EXIF data, missing dependencies")
    return False
    
  if not os.path.exists(image_path):
    print(f"Warning: Image file not found: {image_path}")
    return False
  
  try:
    if dry_run:
      print(f"Would update {image_path}:")
      print(f"  35mm focal length: {focal_length_35mm}mm")
      print(f"  Camera model: {camera_model}")
      return True
    
    # Open image and get existing EXIF data
    img = Image.open(image_path)
    
    # Get existing EXIF data or create new
    exif_dict = {}
    if "exif" in img.info:
      exif_dict = piexif.load(img.info["exif"])
    else:
      exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
    
    # Add camera model to 0th IFD (main image metadata)
    exif_dict["0th"][piexif.ImageIFD.Make] = "OpenMVS"
    exif_dict["0th"][piexif.ImageIFD.Model] = camera_model
    
    # Add focal length information to Exif IFD
    # FocalLengthIn35mmFilm tag
    exif_dict["Exif"][piexif.ExifIFD.FocalLengthIn35mmFilm] = int(round(focal_length_35mm))
    
    # Also add the actual focal length (we'll use the 35mm value as approximation)
    focal_length_rational = (int(focal_length_35mm * 10), 10)  # Convert to rational
    exif_dict["Exif"][piexif.ExifIFD.FocalLength] = focal_length_rational
    
    # Convert back to bytes
    exif_bytes = piexif.dump(exif_dict)
    
    # Save the image with updated EXIF
    img.save(image_path, exif=exif_bytes)
    
    print(f"Updated EXIF for {os.path.basename(image_path)}: {focal_length_35mm}mm, {camera_model}")
    return True
    
  except Exception as e:
    print(f"Error updating EXIF for {image_path}: {e}")
    return False


def process_mvs_scene(mvs_path, images_path, dry_run=False):
  """
  Process an MVS scene and update EXIF data for all images.
  
  Args:
    mvs_path: Path to the MVS interface file
    images_path: Path to the directory containing image files
    dry_run: If True, only print what would be done
  """
  print(f"Loading MVS scene from: {mvs_path}")
  mvs = loadMVSInterface(mvs_path)
  
  if not mvs:
    print("Error: Could not load MVS scene")
    return False
  
  print(f"Loaded MVS scene with {len(mvs['platforms'])} platforms and {len(mvs['images'])} images")
  
  updated_count = 0
  error_count = 0
  
  # Process each image in the scene
  for image_idx, image_info in enumerate(mvs['images']):
    image_name = image_info['name']
    platform_id = image_info['platform_id']
    camera_id = image_info['camera_id']
    
    # Get platform and camera information
    if platform_id >= len(mvs['platforms']):
      print(f"Warning: Invalid platform ID {platform_id} for image {image_name}")
      error_count += 1
      continue
    
    platform = mvs['platforms'][platform_id]
    if camera_id >= len(platform['cameras']):
      print(f"Warning: Invalid camera ID {camera_id} for image {image_name}")
      error_count += 1
      continue
    
    camera = platform['cameras'][camera_id]
    
    # a spherical camera has no focal length (K is angular): nothing to write
    if camera.get('model') == CAMERA_MODEL_SPHERICAL:
      print(f"Warning: skipping spherical camera {camera_id} of platform {platform_id}: {image_name}")
      continue

    # Extract camera parameters (for a BROWN camera K is the pinhole K of the image file, as needed here)
    K = camera['K']  # Intrinsic matrix
    fx = K[0][0]
    fy = K[1][1]
    width = camera.get('width', 0)
    height = camera.get('height', 0)
    
    if width == 0 or height == 0:
      print(f"Warning: No image dimensions for camera {camera_id} in platform {platform_id}")
      error_count += 1
      continue
    
    # Calculate 35mm equivalent focal length
    focal_length_35mm = calculate_35mm_focal_length(fx, fy, width, height)
    
    # Generate camera model name
    platform_name = platform.get('name', f'Platform_{platform_id}')
    camera_name = camera.get('name', f'Camera_{camera_id}')
    camera_model = get_camera_model_name(platform_name, camera_name)
    
    # Find the image file
    image_path = os.path.join(images_path, image_name)
    
    # Try common image extensions if exact name not found
    if not os.path.exists(image_path):
      base_name = os.path.splitext(image_name)[0]
      for ext in ['.jpg', '.jpeg', '.png', '.tiff', '.tif']:
        test_path = os.path.join(images_path, base_name + ext)
        if os.path.exists(test_path):
          image_path = test_path
          break
    
    # Update EXIF data
    if add_exif_to_image(image_path, focal_length_35mm, camera_model, dry_run):
      updated_count += 1
    else:
      error_count += 1
  
  print(f"\nProcessing complete:")
  print(f"  Successfully processed: {updated_count} images")
  print(f"  Errors: {error_count} images")
  
  return error_count == 0


def main():
  parser = ArgumentParser()
  parser.add_argument('-i', '--input', type=str, required=True, 
                     help='Path to the MVS interface archive file')
  parser.add_argument('-p', '--images-path', type=str, required=True,
                     help='Path to the directory containing image files')
  parser.add_argument('--dry-run', action='store_true',
                     help='Print what would be done without actually modifying files')
  args = parser.parse_args()
  
  # Check dependencies first
  if not DEPENDENCIES_AVAILABLE and not args.dry_run:
    print("Error: Missing required dependencies for EXIF modification.")
    print("Install with: pip install pillow piexif")
    print("Or use --dry-run to see what would be done.")
    return 1
  
  # Validate input files
  if not os.path.exists(args.input):
    print(f"Error: MVS file not found: {args.input}")
    return 1
  
  if not os.path.isdir(args.images_path):
    print(f"Error: Images directory not found: {args.images_path}")
    return 1
  
  # Process the MVS scene
  success = process_mvs_scene(args.input, args.images_path, args.dry_run)
  
  return 0 if success else 1


if __name__ == '__main__':
  exit(main())