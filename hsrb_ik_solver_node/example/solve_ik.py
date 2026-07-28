#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
import time

from geometry_msgs.msg import (
    Pose,
    TransformStamped,
)
from moveit_msgs.msg import (
    CollisionObject,
    PlanningSceneWorld,
)
from nav_msgs.msg import OccupancyGrid
import rclpy
from sensor_msgs.msg import (
    JointState,
    PointCloud2,
)
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import tf2_ros
from tmc_manipulation_msgs.srv import SolveIkWithCollision


class PoseBroadcaster:

    def __init__(self, node: rclpy.node.Node) -> None:
        self._broadcaster = tf2_ros.TransformBroadcaster(node)
        self._clock = node.get_clock()

    def broadcast(self, pose: Pose, frame_id: str) -> None:
        msg = TransformStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self._clock.now().to_msg()
        msg.child_frame_id = frame_id
        if pose is not None:
            msg.transform.translation.x = pose.position.x
            msg.transform.translation.y = pose.position.y
            msg.transform.translation.z = pose.position.z
            msg.transform.rotation.x = pose.orientation.x
            msg.transform.rotation.y = pose.orientation.y
            msg.transform.rotation.z = pose.orientation.z
            msg.transform.rotation.w = pose.orientation.w
        self._broadcaster.sendTransform(msg)


class JointStateBroadcaster:

    def __init__(self, node: rclpy.node.Node) -> None:
        self._pub = node.create_publisher(JointState, 'ik_result', 1)

    def broadcast(self, joint_names: list[str], joint_positions: list[float]) -> None:
        msg = JointState()
        msg.name = joint_names
        msg.position = joint_positions
        self._pub.publish(msg)


class PointsEnvironmentBroadcaster:

    def __init__(self, node: rclpy.node.Node) -> None:
        self._pub = node.create_publisher(PointCloud2, 'environment', 1)
        self._clock = node.get_clock()

    def broadcast(self, env: PlanningSceneWorld) -> None:
        header = Header()
        header.stamp = self._clock.now().to_msg()
        header.frame_id = 'odom'

        points = []
        for pose in env.collision_objects[0].primitive_poses:
            points.append((pose.position.x, pose.position.y, pose.position.z))

        msg = point_cloud2.create_cloud_xyz32(header, points)
        self._pub.publish(msg)


class OccupancyGridBroadcaster:

    def __init__(self, node: rclpy.node.Node) -> None:
        self._pub = node.create_publisher(OccupancyGrid, 'obstacle_map', 1)
        self._clock = node.get_clock()

    def broadcast(self, g_map: OccupancyGrid) -> None:
        g_map.header.stamp = self._clock.now().to_msg()
        self._pub.publish(g_map)


def generate_points_environment() -> PlanningSceneWorld:
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.SPHERE
    shape.dimensions.append(0.1)

    collision_object = CollisionObject()
    collision_object.id = 'point_cloud_object'
    for y in range(11):
        pose = Pose()
        pose.position.x = 0.25
        pose.position.y = -0.25 + float(y) * 0.05
        pose.position.z = 0.2
        collision_object.primitive_poses.append(pose)
        collision_object.primitives.append(shape)
    for x in range(11):
        pose = Pose()
        pose.position.x = 1.25 + float(x) * 0.05
        pose.position.y = 0.0
        pose.position.z = 0.3
        collision_object.primitive_poses.append(pose)
        collision_object.primitives.append(shape)

    env = PlanningSceneWorld()
    env.collision_objects.append(collision_object)

    return env


def generate_obstacle_grid_map() -> OccupancyGrid:
    g_map = OccupancyGrid()
    g_map.header.frame_id = 'odom'
    g_map.info.resolution = 0.05
    g_map.info.width = g_map.info.height = 40
    g_map.info.origin.position.y = -1.0
    g_map.data = [0] * (g_map.info.width * g_map.info.height)
    for i in range(10):
        for j in range(20):
            g_map.data[i + 40 * (j + 10)] = 100
    for i in range(20):
        for j in range(10):
            g_map.data[i + 20 + 40 * (j + 15)] = 100
    return g_map


def main():
    rclpy.init()
    node = rclpy.create_node('client_example')

    pose_broadcaster = PoseBroadcaster(node)
    joint_state_broadcaster = JointStateBroadcaster(node)
    environment_broadcaster = PointsEnvironmentBroadcaster(node)
    obstacle_map_broadcaster = OccupancyGridBroadcaster(node)

    points_env = generate_points_environment()
    obstacle_map = generate_obstacle_grid_map()

    evaluation_client = node.create_client(SolveIkWithCollision, 'ik_solver_node/solve_ik_with_collision')
    while not evaluation_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    for x_i in range(10):
        x = 0.5 + 0.2 * float(x_i)
        for i in range(2):
            req = SolveIkWithCollision.Request()
            req.origin_to_hand_goal.position.x = x
            req.origin_to_hand_goal.position.y = 0.0
            if i == 0:
                req.origin_to_hand_goal.position.z = 0.7
                req.origin_to_hand_goal.orientation.x = 0.707
                req.origin_to_hand_goal.orientation.y = 0.0
                req.origin_to_hand_goal.orientation.z = 0.707
                req.origin_to_hand_goal.orientation.w = 0.0
            elif i == 1:
                req.origin_to_hand_goal.position.z = 0.1
                req.origin_to_hand_goal.orientation.x = 1.0
                req.origin_to_hand_goal.orientation.y = 0.0
                req.origin_to_hand_goal.orientation.z = 0.0
                req.origin_to_hand_goal.orientation.w = 0.0
            req.environment = points_env
            req.obstacle_map = obstacle_map

            start = node.get_clock().now()
            future = evaluation_client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            res = future.result()
            end = node.get_clock().now()
            node.get_logger().info(f'Calculation time: {(end - start).nanoseconds / 1e6}[ms]')
            if len(res.ik_results) == 0:
                node.get_logger().error('IK Failure')
                continue
            else:
                node.get_logger().info(f'IK Results Num: {len(res.ik_results)}')

            for _ in range(10):
                pose_broadcaster.broadcast(res.ik_results[0].origin_to_base, 'base_footprint')
                joint_state_broadcaster.broadcast(res.ik_results[0].joint_names, res.ik_results[0].joint_positions)
                environment_broadcaster.broadcast(points_env)
                obstacle_map_broadcaster.broadcast(obstacle_map)
                time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
