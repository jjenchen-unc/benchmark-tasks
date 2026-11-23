"""Conservative clearance from the moving right arm to the table and the two tool-transfer fixtures."""
from functools import lru_cache

import numpy as np
from scipy.spatial import ConvexHull
from yam_arena import analysis
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

import layout as L


def fixture_boxes(stand_pos, stand_rot):
    bx, by, bz = L.STAND_BASE
    half_groove, wall, depth = L.GROOVE_W/2, L.GROOVE_WALL_T, L.GROOVE_D
    local = (
        ("stand floor", (-bx/2, -by/2, -bz), (bx/2, by/2, 0.)),
        ("stand front", (-half_groove-wall, -by/2, 0.), (-half_groove, by/2, depth)),
        ("stand rear", (half_groove, -by/2, 0.), (half_groove+wall, by/2, depth)),
        ("stand backrest", (half_groove, -L.BACKREST_Y, depth), (half_groove+wall, L.BACKREST_Y, L.BACKREST_H)),
    )
    boxes = []
    for name, lo, hi in local:
        lo, hi = np.array(lo), np.array(hi)
        boxes.append((name, stand_pos + stand_rot@((lo+hi)/2), stand_rot, (hi-lo)/2))
    half = np.array(L.HOLSTER_SIZE)/2
    boxes.append(("holster", L.DRIVER_HOLSTER + [0., 0., half[2]], np.eye(3), half))
    return boxes


@lru_cache(maxsize=1)
def arm_vertices():
    """Use each link's outer convex envelope of its actual URDF collision meshes."""
    meshes = analysis.link_hulls()
    result = {}
    for name in ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right"):
        all_verts = np.vstack([h.vertices for h in meshes[name]])
        result[name] = all_verts[ConvexHull(all_verts).vertices]
    return result


def arm_fixture_clearance(q, base, half_gap, boxes):
    """Minimum lower bound on separation. A positive gap on any projection separates the two solids."""
    joints = dict(zip(ARM_JOINTS, q))
    joints.update(dict.fromkeys(FINGER_JOINTS, -half_gap))
    poses = analysis.fk_link_poses_world(joints, base)
    nearest = (float("inf"), "", "")
    for name, vertices in arm_vertices().items():
        transform = poses[name]
        link_axes = transform[:3, :3].T
        world = vertices@link_axes + transform[:3, 3]
        nearest = min(nearest, (float(world[:, 2].min()-L.TOP), name, "table"))
        for fixture, centre, rotation, half in boxes:
            rel = world-centre
            local = rel@rotation
            gap = max(np.max(local.min(0)-half), np.max(-local.max(0)-half))
            if gap < .035:
                # A box-axis test alone can overlap even when a tilted wrist clears a corner.
                axes = np.vstack((link_axes, np.cross(link_axes[:,None,:], rotation.T[None,:,:]).reshape(-1,3),
                                  rel.mean(0)))
                axes = axes[np.linalg.norm(axes,axis=1) > 1e-8]
                axes /= np.linalg.norm(axes,axis=1)[:,None]
                projected = rel@axes.T
                extent = abs(axes@rotation)@half
                gap = max(gap, max(projected.min(0)-extent), max(-projected.max(0)-extent))
            nearest = min(nearest, (float(gap), name, fixture))
    return nearest
