#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Reconstruct a mesh by integrating the given depth-maps into a TSDF volume.

Install:
  pip install open3d numpy tqdm argparse

Example usage:
  python3 MvsDMAP2TSDF.py [-h] --input INPUT [--output OUTPUT] [--voxel_size VOXEL_SIZE] [--truncation_mult TRUNCATION_MULT]
"""

from argparse import ArgumentParser
from glob import glob
from MvsUtils import loadDMAP
from tqdm import tqdm
import numpy as np
import open3d as o3d
import os


def estimate_gsd_from_depth_maps(dmap_paths):
    """
    Estimate the mean GSD (Ground Sampling Distance) from the given depth-maps.

    Args:
      dmap_paths (str): List of depth-map paths

    Returns:
      float: Mean GSD
    """
    # Parse list of depth maps
    mean_gsd = 0.0
    for dmap_path in dmap_paths:
        # Read depth map
        dmap = loadDMAP(dmap_path)

        # Compute the mean depth value
        mean_depth = np.mean(dmap["depth_map"])

        # Compute the GSD
        gsd = mean_depth / dmap["depth_K"][0, 0]
        mean_gsd += gsd
    return mean_gsd / len(dmap_paths)


def create_mesh_from_depth_maps(dmap_paths, voxel_length=0.01, truncation_mult=4.0):
    """
    Reconstruct a mesh from depth maps using TSDF integration.

    Args:
      dmap_paths (str): List of depth-map paths
      voxel_length (float): Size of each voxel in scene units
      truncation_mult (float): Voxel size multiplier to set the truncation value for signed distance function

    Returns:
      open3d.geometry.TriangleMesh: Reconstructed mesh
    """
    # Create TSDF volume
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=truncation_mult * voxel_length,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
    )

    # Parse list of depth maps
    for dmap_path in tqdm(dmap_paths, desc="Integrating depth-maps"):
        # Read depth map
        dmap = loadDMAP(dmap_path)

        # Create RGBD image (using dummy color image)
        depth = o3d.geometry.Image(dmap["depth_map"])
        color = o3d.geometry.Image(np.ones_like(depth))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color,
            depth,
            depth_scale=1.0,  # Adjust based on your depth unit
            depth_trunc=10000.0,  # Maximum depth in meters
            convert_rgb_to_intensity=False,
        )

        # Create camera intrinsic matrix
        assert dmap["depth_K"][0, 1] == 0.0 and dmap["depth_K"][1, 0] == 0.0, "Non-zero skew not supported"
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=dmap["depth_width"],
            height=dmap["depth_height"],
            fx=dmap["depth_K"][0, 0],
            fy=dmap["depth_K"][1, 1],
            cx=dmap["depth_K"][0, 2],
            cy=dmap["depth_K"][1, 2],
        )

        # Get camera pose for this frame
        cam_pose = np.eye(4)
        cam_pose[:3, :3] = dmap["R"]
        cam_pose[:3, 3] = dmap["R"] @ -dmap["C"]

        # Integrate into TSDF volume
        volume.integrate(rgbd, intrinsic, cam_pose)

    # Extract mesh from TSDF volume
    return volume.extract_triangle_mesh()


def dmap2tsdf(input_dir, output_file, voxel_size=0.0, truncation_mult=4.0):
    dmap_paths = sorted(glob(os.path.join(input_dir, "*.dmap")))

    # Estimate GSD if voxel size is not provided
    if voxel_size == 0.0:
        voxel_size = estimate_gsd_from_depth_maps(dmap_paths) * 3.0
        print(f"Estimated voxel size: {voxel_size}")

    # Reconstruct mesh
    mesh = create_mesh_from_depth_maps(dmap_paths, voxel_size, truncation_mult)
    print(f"Reconstructed mesh with {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")

    # Save the mesh
    o3d.io.write_triangle_mesh(output_file, mesh)
    print(f"Mesh saved to {output_file}")


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument(
        "-i", "--input", type=str, required=True, help="Path to the DMAP file directory"
    )
    parser.add_argument(
        "-o", "--output", type=str, default="mesh.ply", help="Path to the reconstructed mesh file",
    )
    parser.add_argument(
        "-x", "--voxel_size", type=float, default=0.0, help="Voxel size for TSDF integration (0 for auto-estimation)",
    )
    parser.add_argument(
        "-t", "--truncation_mult", type=float, default=4.0, help="Truncation multiplier for TSDF integration",
    )
    args = parser.parse_args()
    dmap2tsdf(args.input, args.output, args.voxel_size, args.truncation_mult)
