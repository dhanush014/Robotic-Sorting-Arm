#!/usr/bin/env python3
"""
States (published as std_msgs/String on /sorting_arm/state  (`ros2 topic echo /sorting_arm/state`)):

Fill in function at the bottom for moving to poses

Untested without movement function so might be some bugs
"""

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose

import json

GRIPPER_JOINT_NAME = "left_outer_knuckle_joint"
GRIPPER_TOPIC = "/gripper_controller/commands"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.75 #0.93 = fully closed, will probably change based on testing
GRIPPER_WAIT_TIMEOUT = 5.0
GRIPPER_TOLERANCE = 0.03            # rad

APPROACH_STANDOFF = 0.10

DETECTIONS_TOPIC = "/detected_objects"
DETECTIONS_WAIT_TIMEOUT = 15.0 





def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Minimal RPY -> quaternion (x, y, z, w), avoids a tf_transformations dependency."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = math.pi, yaw: float = 0.0) -> Pose:
    """Default orientation (roll=0, pitch=pi) points straight down, rotate z to change gripper angle."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def offset_pose_z(pose: Pose, dz: float) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = pose.position.x, pose.position.y, pose.position.z + dz
    p.orientation = pose.orientation
    return p


@dataclass
class ObjectTask:
    object_id: str
    pick_pose: Pose
    place_pose: Pose
    gripper_close: float = GRIPPER_CLOSE



COLOR_DROP_POINTS = {"red": make_pose(0.45, -0.20, 0.04),"blue": make_pose(0.15, -0.20, 0.04),}
OBJ1_POSE = make_pose(-0.45, 0.20, 0.85)
OBJ2_POSE = make_pose(-0.15, 0.20, 0.85)
OBJ3_POSE = make_pose(0.15, 0.20, 0.85)
OBJ4_POSE = make_pose(0.45, 0.20, 0.85)


class PickPlaceNode(Node):
    def __init__(self, move_to_pose_fn: Optional[Callable[[Pose], bool]] = None):
        super().__init__("state_machine")
        cb_group = ReentrantCallbackGroup()

        self.state_pub = self.create_publisher(String, "/sorting_arm/state", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10, callback_group=cb_group)

        self.last_joint_state: Optional[JointState] = None

        #subscribe to Krish's topic
        detections_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, DETECTIONS_TOPIC, self._detections_cb, detections_qos, callback_group=cb_group
        )

        self.detected_objects: Optional[list] = None
        if move_to_pose_fn is None:
            raise ValueError("State machine node requires a move_to_pose_fn")
        self.move_to_pose_fn: Callable[[Pose], bool] = move_to_pose_fn

    def _set_state(self, state: str):
        self.get_logger().info(f"[state] {state}")
        self.state_pub.publish(String(data=state))
 
    def _joint_state_cb(self, msg: JointState):
        self.last_joint_state = msg
 
    def _detections_cb(self, msg: String):
        try:
            self.detected_objects = json.loads(msg.data)
            self.get_logger().info(f"Received {len(self.detected_objects)} detections")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Could not parse {DETECTIONS_TOPIC}: {e}")
 
    def _wait_for_detections(self, timeout: float = DETECTIONS_WAIT_TIMEOUT) -> bool:
        start = time.time()
        while self.detected_objects is None and time.time() - start < timeout:
            time.sleep(0.05)
        return self.detected_objects is not None

    # for gripper control:
    def _command_gripper(self, position: float) -> bool:
        msg = Float64MultiArray()
        msg.data = [position, position, position, position, position, position]
        self.gripper_pub.publish(msg)
        return self._wait_for_gripper(position)

    def _wait_for_gripper(self, target: float) -> bool:
        start = time.time()
        while time.time() - start < GRIPPER_WAIT_TIMEOUT:
            js = self.last_joint_state
            if js is not None and GRIPPER_JOINT_NAME in js.name:
                idx = js.name.index(GRIPPER_JOINT_NAME)
                if abs(js.position[idx] - target) < GRIPPER_TOLERANCE:
                    return True
            time.sleep(0.05)
        self.get_logger().warn("Timed out waiting for gripper to reach target; continuing anyway")
        return False


    def run_task(self, task: ObjectTask) -> bool:

        approach_pick = offset_pose_z(task.pick_pose, APPROACH_STANDOFF)
        approach_place = offset_pose_z(task.place_pose, APPROACH_STANDOFF)

        steps = [
            (f"PICK_APPROACH:{task.object_id}", lambda: self.move_to_pose_fn(approach_pick)),
            (f"PICK_DESCEND:{task.object_id}", lambda: self.move_to_pose_fn(task.pick_pose)),
            (f"GRIPPER_CLOSE:{task.object_id}", lambda: self._command_gripper(task.gripper_close)),
            (f"PICK_RETREAT:{task.object_id}", lambda: self.move_to_pose_fn(approach_pick)),
            (f"PLACE_APPROACH:{task.object_id}", lambda: self.move_to_pose_fn(approach_place)),
            (f"PLACE_DESCEND:{task.object_id}", lambda: self.move_to_pose_fn(task.place_pose)),
            (f"GRIPPER_OPEN:{task.object_id}", lambda: self._command_gripper(GRIPPER_OPEN)),
            (f"PLACE_RETREAT:{task.object_id}", lambda: self.move_to_pose_fn(approach_place)),
        ]

        for state_name, step in steps:
            self._set_state(state_name)
            if not step():
                self._set_state(f"ERROR:{state_name}")
                return False
        return True


    def run(self):
        self._set_state("INIT")
        if not self._wait_for_detections():
            self.get_logger().error(f"No message on {DETECTIONS_TOPIC} within timeout, aborting")
            self._set_state("ERROR:WAIT_FOR_DETECTIONS")
            return

        place_pose = {}
        for i, det in enumerate(self.detected_objects or []):
            color = det.get("color", "unknown")
            place_pose[i] = COLOR_DROP_POINTS.get(color)

        # Obbject poses
        OBJECTS: List[ObjectTask] = [
            ObjectTask(
                object_id="obj_1",
                pick_pose=OBJ1_POSE,
                place_pose=place_pose[0],
            ),
            ObjectTask(
                object_id="obj_2",
                pick_pose=OBJ2_POSE,
                place_pose=place_pose[1],
            ),
            ObjectTask(
                object_id="obj_3",
                pick_pose=OBJ3_POSE,
                place_pose=place_pose[2],
            ),
            ObjectTask(
                object_id="obj_4",
                pick_pose=OBJ4_POSE,
                place_pose=place_pose[3],
            ),
        ]

        for task in OBJECTS:
            self.get_logger().info(f"--- Starting {task.object_id} ---")
            if not self.run_task(task):
                self.get_logger().error(f"Aborting after failure on {task.object_id}")
                return

        self._set_state("DONE")


def main(move_to_pose_fn: [Callable[[Pose], bool]] = None):
    rclpy.init()
    node = PickPlaceNode(move_to_pose_fn=move_to_pose_fn)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main(move_to_pose_fn="""DHANUSH FUNCTION HERE I THINK""") #needs to use a geometry_msgs.msg.Pose (can change if you want)
    

