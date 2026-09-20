#!/usr/bin/env python3
"""Send Cerberus's neutral standing pose once the trajectory controller is ready."""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint\

from rclpy.duration import Duration


JOINT_NAMES = [
    "fr_hip_joint", "fl_hip_joint", "br_hip_joint", "bl_hip_joint",
    "fr_thigh_joint", "fl_thigh_joint", "br_thigh_joint", "bl_thigh_joint",
    "fr_knee_joint", "fl_knee_joint", "br_knee_joint", "bl_knee_joint",
]


class StandPose(Node):
    def __init__(self):
        super().__init__("stand_pose")
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.command_sent = False
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        if not self.command_sent:
            if self.trajectory_publisher.get_subscription_count() == 0:
                return

            trajectory = JointTrajectory()
            trajectory.joint_names = JOINT_NAMES
            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.82, -0.82, 0.82, -0.82, -1.2,  1.2, -1.2,  1.2] 
            point.time_from_start.sec = 4
            point.time_from_start = Duration(seconds=1).to_msg()      
            trajectory.points = [point]

            # A zero stamp means "start immediately" in simulation time.
            self.trajectory_publisher.publish(trajectory)
            self.command_sent = True
            self.get_logger().info("Neutral stand posture sent over three seconds.")


def main():
    rclpy.init()
    node = StandPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
