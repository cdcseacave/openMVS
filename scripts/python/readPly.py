#!/usr/bin/env python3
"""Shows how to use the Rerun SDK to log a textured 3D mesh from a .ply file and its associated texture."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import cast

import numpy as np
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
import trimesh
from PIL import Image  # pip install pillow
import trimesh.transformations as tf

def load_ply_with_texture(mesh_path: Path, texture_path: Path) -> trimesh.Trimesh:
    print(f"Loading mesh {mesh_path} with texture {texture_path}…")
    mesh = trimesh.load(mesh_path)

    # Ensure it's a Trimesh object, not a Scene
    if isinstance(mesh, trimesh.Scene):
        raise ValueError("Loaded file is a scene, not a single mesh. Please load a single .ply mesh.")

    # Load texture image
    texture_image = Image.open(texture_path)
    texture_image = np.array(texture_image)

    # Apply texture to the mesh
    #if not mesh.visual.uv:
        #raise ValueError("Mesh does not contain UV coordinates for texturing.")
    
    mesh.visual = trimesh.visual.TextureVisuals(uv=mesh.visual.uv, image=texture_image)
    
    return mesh

def apply_rotation(mesh: trimesh.Trimesh, angle_degrees: float, axis: np.ndarray) -> trimesh.Trimesh:
    # Create a rotation matrix
    rotation_matrix = tf.rotation_matrix(np.radians(angle_degrees), axis)
    
    # Apply the rotation to the mesh
    mesh.apply_transform(rotation_matrix)
    
    return mesh

def log_mesh(mesh: trimesh.Trimesh, path: str) -> None:
    vertex_colors = None
    vertex_texcoords = None
    albedo_texture = None

    # Extract texture coordinates and image
    try:
        vertex_texcoords = mesh.visual.uv
        vertex_texcoords[:, 1] = 1.0 - vertex_texcoords[:, 1]  # Flip V coordinate
        albedo_texture = mesh.visual.material.image  # Using PIL Image
    except Exception as e:
        print(f"Error processing texture coordinates or image: {e}")

    # Log the mesh
    rr.log(
        path,
        rr.Mesh3D(
            vertex_positions=mesh.vertices,
            vertex_colors=vertex_colors,
            vertex_normals=mesh.vertex_normals,  # type: ignore[arg-type]
            vertex_texcoords=vertex_texcoords,
            albedo_texture=albedo_texture,
            triangle_indices=mesh.faces,
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Logs a textured 3D mesh from a .ply file using the Rerun SDK."
    )
    parser.add_argument(
        "--mesh-path",
        type=Path,
        required=True,
        help="Path to the .ply mesh file",
    )
    parser.add_argument(
        "--texture-path",
        type=Path,
        required=True,
        help="Path to the texture image (e.g., .png) file",
    )
    rr.script_add_args(parser)
    args = parser.parse_args()

    mesh_path = args.mesh_path
    texture_path = args.texture_path

    mesh = load_ply_with_texture(mesh_path, texture_path)
    mesh = apply_rotation(mesh, 180, np.array([1, 0, 0]))
    blueprint = rrb.Horizontal(
        rrb.Spatial3DView(name="Mesh", origin="/world"),
        rrb.TextDocumentView(name="Description", origin="/description"),
        column_shares=[3, 1],
    )

    rr.script_setup(args, "rerun_example_textured_mesh", default_blueprint=blueprint)
    rr.log("description", rr.TextDocument("Textured mesh logging example", media_type=rr.MediaType.MARKDOWN), timeless=True)

    # Assuming we are logging a single mesh
    rr.log("world", rr.ViewCoordinates.RUB, static=True)
    log_mesh(mesh, "world/mesh")

    rr.script_teardown(args)


if __name__ == "__main__":
    main()
