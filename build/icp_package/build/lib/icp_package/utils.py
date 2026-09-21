"""Helpers shared by scan_merge and scan_match"""

import math

import numpy as np


def euler_rotation_matrix(roll, pitch, yaw):
    """Build the Z-Y-X rotation matrix (Yaw @ Pitch @ Roll)."""
    rot_yaw = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    rot_pitch = np.array(
        [
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ]
    )
    rot_roll = np.array(
        [
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)],
        ]
    )
    return rot_yaw @ rot_pitch @ rot_roll
