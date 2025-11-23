#!/usr/bin/env python3
import numpy as np
import trimesh
import os
import json

# 1) Edit this to point at your meshes – either absolute paths,
MESH_FILES = {
  "link_base": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link_base.stl",
  "link1": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link1.stl",
  "link2": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link2.stl",
  "link3": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link3.stl",
  "link4": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link4.stl",
  "link5": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link5.stl",
  "link6": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link6.stl",
  "link7": "src/xarm_ros2/xarm_description/meshes/xarm7/visual/link7.stl",
}

# 2) Grid resolution (meters)
DX = 0.005

def compute_sdf_for_mesh(path, dx):
    mesh = trimesh.load(path)
    mn, mx = mesh.bounds
    xs = np.arange(mn[0], mx[0], dx)
    ys = np.arange(mn[1], mx[1], dx)
    zs = np.arange(mn[2], mx[2], dx)
    grid = np.stack(np.meshgrid(xs, ys, zs, indexing='xy'), -1)
    pts = grid.reshape(-1,3)
    sdf = trimesh.proximity.signed_distance(mesh, pts)
    return {
        "bounds": [mn.tolist(), mx.tolist()],
        "dx": dx,
        "points": pts.tolist(),
        "sdf": sdf.tolist(),
    }

def main():
    out = {}
    for link, path in MESH_FILES.items():
        print(f"Processing {link}…")
        if not os.path.isfile(path):
            print(f"  ✗ file not found: {path}")
            continue
        out[link] = compute_sdf_for_mesh(path, DX)
    with open("xarm7_sdf.json", "w") as f:
        json.dump(out, f)
    print("Saved xarm7_sdf.json")

if __name__ == "__main__":
    main()
