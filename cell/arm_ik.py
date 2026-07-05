# -*- coding: utf-8 -*-
"""Closed-form kinematics for the 4-axis palletizer arm.

Chain (MuJoCo conventions, hinge axes):
  j1 yaw   about +z at base
  j2 shoulder pitch about local +y (positive pitches the link DOWN, R_y)
  j3 elbow    pitch about local +y
  j4 wrist    pitch about local +y — commanded to -(q2+q3) so the tool stays vertical

Frames: shoulder pivot S = (bx, by, SHOULDER_Z). With q = 0 the arm points
along local +x (rotated by q1 in the world).
Wrist W_local = [L1 c2 + L2 c23, 0, -(L1 s2 + L2 s23)];  TCP = W - [0,0,tool_len].
"""
import numpy as np

from cell.params import ARM, SHOULDER_Z

BX, BY = ARM["base"]
L1, L2, TOOL = ARM["L1"], ARM["L2"], ARM["tool_len"]


def fk(q):
    """q = (q1, q2, q3, q4) -> (tcp_xyz, wrist_xyz). q4 is ignored for position
    (tool vertical is enforced by q4 = -(q2+q3))."""
    q1, q2, q3, _ = q
    c2, s2 = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
    r = L1 * c2 + L2 * c23
    z = -(L1 * s2 + L2 * s23)
    wrist = np.array([BX + r * np.cos(q1), BY + r * np.sin(q1), SHOULDER_Z + z])
    tcp = wrist - np.array([0.0, 0.0, TOOL])
    return tcp, wrist


def ik(tcp_xyz):
    """TCP target -> q = (q1, q2, q3, q4), elbow-up branch. Raises ValueError
    if out of reach."""
    x, y, z = tcp_xyz
    wx, wy, wz = x, y, z + TOOL
    dx, dy = wx - BX, wy - BY
    q1 = np.arctan2(dy, dx)
    r = np.hypot(dx, dy)
    h = -(wz - SHOULDER_Z)             # planar "down" coordinate
    d2 = r * r + h * h
    D = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if not (-1.0 <= D <= 1.0):
        raise ValueError(f"target {tcp_xyz} out of reach (D={D:.3f})")
    best = None
    for q3 in (np.arccos(D), -np.arccos(D)):
        q2 = np.arctan2(h, r) - np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))
        # elbow height in world (for the elbow-up preference)
        elbow_z = SHOULDER_Z - L1 * np.sin(q2)
        cand = (elbow_z, (q1, q2, q3, -(q2 + q3)))
        if best is None or cand[0] > best[0]:
            best = cand
    q = np.array(best[1])
    # verify
    tcp, _ = fk(q)
    if not np.allclose(tcp, tcp_xyz, atol=1e-9):
        raise AssertionError(f"IK/FK mismatch: {tcp} vs {tcp_xyz}")
    return q


def unwrap_yaw(q1_target, q1_current):
    """Choose the q1 representation closest to the current angle (shortest swing)."""
    d = q1_target - q1_current
    while d > np.pi:
        d -= 2 * np.pi
    while d < -np.pi:
        d += 2 * np.pi
    return q1_current + d
