import os
import tempfile
import re
import subprocess
import xml.etree.ElementTree as ET

import pybullet as p
import pybullet_data

class SelfCollisionChecker:
    """
    Loads a URDF into a PyBullet DIRECT client and checks for self-collisions.
    Automatically sanitizes package:// URIs, fixes double suffixes, and strips visuals
    to avoid missing-mesh failures.
    """
    def __init__(self, urdf_path, fixed_base=True, exclude_parent_child=True):
        # sanitize/expand URDF into a temp file
        self._temp_urdf = self._sanitize_urdf(urdf_path)

        # Connect headless
        self.cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        flags = p.URDF_USE_SELF_COLLISION
        if exclude_parent_child:
            flags |= p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS

        # Load sanitized URDF
        self.robot = p.loadURDF(
            self._temp_urdf,
            useFixedBase=fixed_base,
            flags=flags
        )
        self.num_joints = p.getNumJoints(self.robot)

        # Build name <-> index maps and list of movable joint indices
        self.joint_name_to_index = {}
        self.movable_joint_indices = []
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            name = info[1].decode("utf-8")
            joint_type = info[2]
            self.joint_name_to_index[name] = i
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL, p.JOINT_PLANAR):
                self.movable_joint_indices.append(i)

        # Precompute candidate link pairs for collision queries (skip identical)
        self._link_pairs = [(i, j) for i in range(self.num_joints) for j in range(i + 1, self.num_joints)]

        # Turn off dynamics influence for pure geometry checking
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(0)

    def _sanitize_urdf(self, urdf_path):
        # Read original URDF
        with open(os.path.expanduser(urdf_path), 'r') as f:
            xml = f.read()

        # Resolve package://xarm_description to absolute path once
        try:
            proc = subprocess.run(
                ["ros2", "pkg", "prefix", "xarm_description"],
                capture_output=True,
                text=True,
                check=True
            )
            pkg_prefix = proc.stdout.strip()
            xml = re.sub(
                r"package://xarm_description/",
                "/home/danielrsouza/dev_ws/src/xarm_ros2/xarm_description/",
                xml
            )
        except Exception:
            # fallback: leave package:// URIs; loader may fail later
            pass

        # Strip visual elements to avoid missing mesh issues (collision doesn't need them)
        try:
            root = ET.fromstring(xml)
            for link in root.findall("link"):
                for vis in link.findall("visual"):
                    link.remove(vis)
            xml = ET.tostring(root, encoding='unicode')
        except Exception:
            # if parsing fails, continue with raw xml
            pass

        # Write sanitized content to a temp file
        fd, temp_path = tempfile.mkstemp(suffix=".urdf")
        with os.fdopen(fd, "w") as tmp:
            tmp.write(xml)
        return temp_path

    def set_joint_positions(self, joint_angles):
        """
        joint_angles: list (ordered according to get_movable_joint_names()) or dict name->value.
        """
        if isinstance(joint_angles, dict):
            for name, val in joint_angles.items():
                if name not in self.joint_name_to_index:
                    raise KeyError(f"Joint '{name}' not in URDF.")
                idx = self.joint_name_to_index[name]
                p.resetJointState(self.robot, idx, val)
        else:
            if len(joint_angles) != len(self.movable_joint_indices):
                raise ValueError(f"Expected {len(self.movable_joint_indices)} values, got {len(joint_angles)}.")
            for idx, val in zip(self.movable_joint_indices, joint_angles):
                p.resetJointState(self.robot, idx, val)

    def check_self_collision(self, distance_threshold=0.0):
        collisions = []
        for i in range(self.num_joints):
            for j in range(i + 1, self.num_joints):
                pts = p.getClosestPoints(self.robot, self.robot, distance_threshold, i, j)
                collisions.extend(pts)
        return collisions


    def in_self_collision(self, distance_threshold=0.0):
        """
        Boolean: any self-collision within threshold?
        Early exit on first detection.
        """
        for i, j in self._link_pairs:
            pts = p.getClosestPoints(self.robot, self.robot, distance_threshold, i, j)
            if pts:
                return True
        return False

    def minimum_distance(self):
        """
        Returns the minimal (worst-case) distance among all self link pairs.
        Negative means penetration.
        """
        min_d = float("inf")
        for i, j in self._link_pairs:
            # query with a large threshold to get the closest
            pts = p.getClosestPoints(self.robot, self.robot, 1.0, i, j)
            for pt in pts:
                if pt["distance"] < min_d:
                    min_d = pt["distance"]
        if min_d == float("inf"):
            # no proximity info, return a large positive number
            return 3.0
        return min_d

    def get_movable_joint_names(self):
        names = []
        for idx in self.movable_joint_indices:
            info = p.getJointInfo(self.robot, idx)
            names.append(info[1].decode("utf-8"))
        return names

    def disconnect(self):
        try:
            p.disconnect(self.cid)
        except Exception:
            pass
        # clean temp urdf
        try:
            if hasattr(self, "_temp_urdf") and os.path.exists(self._temp_urdf):
                os.remove(self._temp_urdf)
        except Exception:
            pass

    def __del__(self):
        self.disconnect()
