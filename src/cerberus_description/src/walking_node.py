#!/usr/bin/env python3
"""Cerberus trot gait: /cmd_vel (forward/back, strafe, turn CW/CCW) -> foot paths -> IK -> JTC.

Run (after the sim and controllers are up):
  ros2 run cerberus_description walk_node.py --ros-args -p use_sim_time:=true
  ros2 run teleop_twist_keyboard teleop_twist_keyboard      # drive it

Standing is the zero-velocity case: with no command the robot settles into the crouch and
stops stepping. Use -p march_in_place:=true to keep stepping while standing.

Geometry: thigh and knee axes are parallel, so each leg is an exact planar 2-link in the
x-z plane; the hip joint rotates that plane about the body x axis (needed for turning and
strafing). Constants come from cerberus_final.urdf and the IK was checked against the
URDF forward kinematics.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEGS = ("fr", "fl", "br", "bl")
JOINT_NAMES = [
    "fr_hip_joint", "fl_hip_joint", "br_hip_joint", "bl_hip_joint",
    "fr_thigh_joint", "fl_thigh_joint", "br_thigh_joint", "bl_thigh_joint",
    "fr_knee_joint", "fl_knee_joint", "br_knee_joint", "bl_knee_joint",
]
HIP_I = {"fr": 0, "fl": 1, "br": 2, "bl": 3}
THIGH_I = {"fr": 4, "fl": 5, "br": 6, "bl": 7}
KNEE_I = {"fr": 8, "fl": 9, "br": 10, "bl": 11}

# ---- leg geometry, taken from cerberus_final.urdf (metres, base_link frame) ----
# Hip joint origin (axis along +/-x), thigh joint origin (x, z), foot y at hip angle 0.
HIP = {"fr": (0.110, 0.060, 0.025), "fl": (0.110, -0.060, 0.025),
       "br": (-0.110, 0.060, 0.025), "bl": (-0.110, -0.060, 0.025)}
THIGH_XZ = {"fr": (0.1288, 0.025), "fl": (0.1288, 0.025),
            "br": (-0.1287, 0.025), "bl": (-0.1288, 0.025)}
FOOT_Y0 = {"fr": 0.1392, "fl": -0.1391, "br": 0.1392, "bl": -0.1391}
HIP_SIGN = {"fr": 1, "fl": 1, "br": -1, "bl": -1}      # hip axis +x (front) / -x (rear)
PLANE_SIGN = {"fr": 1, "fl": -1, "br": 1, "bl": -1}    # thigh/knee axis +y (right) / -y (left)

# Thigh->knee (L1) and knee->foot (L2) in the leg's x-z plane at zero joint angles.
L1 = np.array([0.0223, -0.1243])
L2 = np.array([0.0503, -0.0850])
_A = float(L1 @ L2)
_B = float(L1[0] * L2[1] - L1[1] * L2[0])
_RHO = math.hypot(_A, _B)
_PHI = math.atan2(_B, _A)
_C0 = (float(L1 @ L1) + float(L2 @ L2)) / 2.0

# Fixed crouch (feet 45 mm behind "foot under the hip"), right-leg convention.
CROUCH_THIGH = 1.065
CROUCH_KNEE = -1.128


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, s], [-s, c]])


def planar_fk(t1, t2):
    """Foot (x, z) from the thigh joint, right-leg angle convention."""
    return _rot(t1) @ (L1 + _rot(t2) @ L2)


def planar_ik(px, pz, knee_ref=CROUCH_KNEE):
    """Return (thigh, knee), right-leg convention. Raises ValueError if out of reach."""
    c = ((px * px + pz * pz) / 2.0 - _C0) / _RHO
    if abs(c) > 1.0:
        raise ValueError("foot target out of reach")
    a = math.acos(c)
    t2 = min((_wrap(_PHI + a), _wrap(_PHI - a)), key=lambda t: abs(t - knee_ref))
    v = L1 + _rot(t2) @ L2
    t1 = _wrap(math.atan2(v[1], v[0]) - math.atan2(pz, px))
    return t1, t2


def leg_ik(leg, target):
    """Foot target (x, y, z) in base_link -> (hip, thigh, knee) joint angles."""
    hx, hy, hz = HIP[leg]
    tx, tz = THIGH_XZ[leg]
    x, y, z = target
    d = FOOT_Y0[leg] - hy                     # lateral offset of the leg plane from the hip axis
    ty, tzr = y - hy, z - hz
    r2 = ty * ty + tzr * tzr - d * d
    if r2 <= 1e-9:
        raise ValueError("foot target too close to the hip axis")
    w = -math.sqrt(r2)                        # leg-plane z relative to the hip axis
    a = math.atan2(tzr, ty) - math.atan2(w, d)
    hip = HIP_SIGN[leg] * _wrap(a)
    t1, t2 = planar_ik(x - tx, (hz + w) - tz)
    s = PLANE_SIGN[leg]
    return hip, s * t1, s * t2


# ---- neutral stance ----
def neutral_feet():
    px, pz = planar_fk(CROUCH_THIGH, CROUCH_KNEE)
    return {leg: np.array([THIGH_XZ[leg][0] + px, FOOT_Y0[leg], THIGH_XZ[leg][1] + pz])
            for leg in LEGS}


NEUTRAL = neutral_feet()
_centre = np.mean([NEUTRAL[l][:2] for l in LEGS], axis=0)
REL = {leg: NEUTRAL[leg][:2] - _centre for leg in LEGS}      # foot position about the yaw centre


def crouch_joints():
    q = [0.0] * 12
    for leg in LEGS:
        s = PLANE_SIGN[leg]
        q[THIGH_I[leg]] = s * CROUCH_THIGH
        q[KNEE_I[leg]] = s * CROUCH_KNEE
    return q


# ---- gait ----
def swing_height(s):
    """Up/down profile: 0 -> 1 -> 0 over s in [0, 1], zero slope at both ends."""
    return 0.5 * (1.0 - math.cos(2.0 * math.pi * s))


def foot_advance(s):
    """Back/front profile: 0 -> 1 over s in [0, 1], zero slope at both ends."""
    return 0.5 * (1.0 - math.cos(math.pi * s))


# Phase offsets for diagonal-pair trot: front-rear pair leads, middle follows.
PHASE_OFFSET = {"fr": 0.0, "bl": 0.0, "fl": 0.5, "br": 0.5}


def stride_vectors(vx, vy, wz, t_stance):
    """Forward travel of each foot during swing (= backward travel during stance)."""
    return {leg: np.array([vx - wz * REL[leg][1], vy + wz * REL[leg][0]]) * t_stance
            for leg in LEGS}


def foot_target(leg, phase, stride, height):
    ph = (phase + PHASE_OFFSET[leg]) % 1.0
    if ph < 0.5:                               # swing
        s = ph / 0.5
        frac = -0.5 + foot_advance(s)
        dz = height * swing_height(s)
    else:                                      # stance
        frac = 0.5 - (ph - 0.5) / 0.5
        dz = 0.0
    n = NEUTRAL[leg]
    return np.array([n[0] + frac * stride[0], n[1] + frac * stride[1], n[2] + dz])


def joints_at(phase, stride, height):
    q = [0.0] * 12
    for leg in LEGS:
        hip, th, kn = leg_ik(leg, foot_target(leg, phase, stride[leg], height))
        q[HIP_I[leg]], q[THIGH_I[leg]], q[KNEE_I[leg]] = hip, th, kn
    return q


class Gait:
    """cmd_vel -> filtered command -> foot paths -> joint targets (pure Python, testable)."""

    def __init__(self, step_time=0.12, height=0.05, max_stride=0.06,
                 march_in_place=False, lin_acc=2.0, ang_acc=5.0):
        self.step_time = step_time            # seconds per half cycle (= stance time)
        self.cycle = 2.0 * step_time
        self.height = height
        self.max_stride = max_stride
        self.march_in_place = march_in_place
        self.lin_acc, self.ang_acc = lin_acc, ang_acc
        self.phase = 0.0                      # phase of the (fr, bl) pair
        self.cmd = np.zeros(3)                # filtered (vx, vy, wz)
        self.target = np.zeros(3)

    def set_target(self, vx, vy, wz):
        self.target = np.array([vx, vy, wz], dtype=float)

    # -- command shaping --
    def _effective(self):
        """Filtered command scaled so no foot travels more than max_stride per step."""
        strides = stride_vectors(*self.cmd, self.step_time)
        m = max(float(np.linalg.norm(s)) for s in strides.values())
        k = 1.0 if m <= self.max_stride else self.max_stride / m
        return self.cmd * k

    def _wants_motion(self, eff):
        strides = stride_vectors(*eff, self.step_time)
        return self.march_in_place or max(float(np.linalg.norm(s)) for s in strides.values()) > 1e-4

    @staticmethod
    def _advance(phase, step, want):
        """Advance the phase; when not wanted, stop exactly at the next 'all feet down' point."""
        if want:
            return (phase + step) % 1.0
        # Fixed: use epsilon comparison to handle floating-point drift
        if abs(phase) < 1e-9 or abs(phase - 0.5) < 1e-9:
            return phase
        nxt = 0.5 if phase < 0.5 else 1.0
        new = phase + step
        return (nxt % 1.0) if new >= nxt else new

    def update(self, dt):
        lim = np.array([self.lin_acc, self.lin_acc, self.ang_acc]) * dt
        self.cmd = self.cmd + np.clip(self.target - self.cmd, -lim, lim)
        eff = self._effective()
        self.phase = self._advance(self.phase, dt / self.cycle, self._wants_motion(eff))
        return eff

    def joint_points(self, count=3, ahead=0.04):
        """Joint targets for the next `count` points, `ahead` seconds apart (None if unreachable)."""
        eff = self._effective()
        want = self._wants_motion(eff)
        strides = stride_vectors(*eff, self.step_time)
        phase, pts = self.phase, []
        try:
            for _ in range(count):
                phase = self._advance(phase, ahead / self.cycle, want)
                pts.append(joints_at(phase, strides, self.height))
        except ValueError:
            return None
        return pts


class GaitNode(Node):
    def __init__(self):
        super().__init__("walking_node")
        self.declare_parameter("step_time", 0.12)        # s per half cycle (stance = swing time)
        self.declare_parameter("step_height", 0.05)      # m of foot lift
        self.declare_parameter("max_stride", 0.06)       # m a foot may travel per step
        self.declare_parameter("cmd_timeout", 0.5)       # s without /cmd_vel -> stop (<=0: never)
        self.declare_parameter("march_in_place", False)
        self.declare_parameter("rate", 50.0)             # Hz of trajectory updates
        self.declare_parameter("lin_acc", 2.0)           # m/s^2 linear acceleration limit
        self.declare_parameter("ang_acc", 5.0)           # rad/s^2 angular acceleration limit
        self.declare_parameter("ahead", 0.02)           # s between trajectory points
        self.declare_parameter("traj_count", 5)          # number of trajectory points to send
        p = self.get_parameter
        self.gait = Gait(step_time=float(p("step_time").value),
                         height=float(p("step_height").value),
                         max_stride=float(p("max_stride").value),
                         march_in_place=bool(p("march_in_place").value),
                         lin_acc=float(p("lin_acc").value),
                         ang_acc=float(p("ang_acc").value))
        self.cmd_timeout = float(p("cmd_timeout").value)
        self.rate = float(p("rate").value)
        self.ahead = float(p("ahead").value)
        self.traj_count = int(p("traj_count").value)

        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.start_after = None
        self.last_tick = None
        self.last_cmd_time = None
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def on_cmd(self, msg):
        self.gait.set_target(msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def send(self, points):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        for k, q in enumerate(points):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.time_from_start = Duration(seconds=self.ahead * (k + 1)).to_msg()
            traj.points.append(pt)
        self.pub.publish(traj)

    def tick(self):
        if self.pub.get_subscription_count() == 0:
            return
        now = self.get_clock().now()

        # First settle into the crouch, then start walking.
        if self.start_after is None:
            traj = JointTrajectory()
            traj.joint_names = JOINT_NAMES
            pt = JointTrajectoryPoint()
            pt.positions = crouch_joints()
            pt.time_from_start = Duration(seconds=2.0).to_msg()
            traj.points = [pt]
            self.pub.publish(traj)
            self.start_after = now + Duration(seconds=2.0)
            self.get_logger().info("Going to crouch, then ready for /cmd_vel.")
            return
        if now < self.start_after:
            return

        dt = 0.0
        if self.last_tick is not None:
            dt = min(max((now - self.last_tick).nanoseconds * 1e-9, 0.0), 0.1)
        self.last_tick = now

        if (self.cmd_timeout > 0.0 and self.last_cmd_time is not None
                and (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout):
            self.gait.set_target(0.0, 0.0, 0.0)

        self.gait.update(dt)
        points = self.gait.joint_points(self.traj_count, self.ahead)
        if points is None:
            self.get_logger().warn("Foot target out of reach - skipping update.",
                                   throttle_duration_sec=2.0)
            return
        self.send(points)


def main():
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()