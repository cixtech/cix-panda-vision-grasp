#!/usr/bin/env python3
"""
Automatic grasp controller (started by gazebo.launch as node ``commander``).

Data flow:
  GraphExecuter click -> /target_point [x, y, angle]
  -> listener_callback() -> _run_grasp_sequence() runs 7 steps in sequence
  -> /grasp/status publishes idle | running (failed only if /reset_arm cannot reach place)

Arm motion helpers in this file:
  - plan_and_execute()  : OMPL plan + trajectory execution (arm/gripper)
  - move_to()           : Cartesian arm motion; linear=True uses Pilz LIN; Step 1 allows z band on retry
  - _gripper_close_sync(): fire close, wait 2s, then j1+j2 stable 0.5s counts as grasped
  - _abort_grasp_and_return_place(): on failure, return to place (gripper unchanged during motion), then open
  - _startup_home_worker(): move above place point (place_x, place_y, height) after startup
  - _run_grasp_sequence(): lift to start height, transport to place, release; lift failure returns to place
"""

import glob
import math
import os
import shutil
import subprocess
import sys
import time
import threading

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.kinematic_constraints import construct_joint_constraint


# SRDF tip_link for panda_arm — Cartesian goals must use the gripper link.
EE_LINK = "panda_hand"
GRIPPER_OPEN = 0.04
GRIPPER_DESCENT_OPEN = 0.04
GRIPPER_CLOSE = 0.003
GRIPPER_OPEN_MAX_EFFORT = 40.0
GRIPPER_OPEN_SETTLE_S = 0.5
GRIPPER_OPEN_TOLERANCE = 0.0015
GRIPPER_CLOSE_AFTER_START_S = 2.0
GRIPPER_GRASP_STABLE_S = 0.5
GRIPPER_GRASP_STABLE_TOLERANCE = 0.001
GRIPPER_GRASP_DETECT_TIMEOUT_S = 5.0
FINGER_JOINT1 = "panda_finger_joint1"
FINGER_JOINT2 = "panda_finger_joint2"
GRIPPER_CMD_TIMEOUT_S = 10.0
GRIPPER_CMD_ACTION = "/panda_hand_controller/gripper_cmd"
FIXED_EE_Z_TOLERANCE = 0.035
CARRY_TRANSPORT_MIN_EE_Z = 0.19
# Path constraint: EE x in panda_link0 must stay >= MIN_EE_X (non-return motions).
MIN_EE_X = 0.0
MIN_EE_AXIS_BOX_SPAN = 3.0
# Step 6 transport legs: fixed EE height (see _return_fixed_z_trials).
RETURN_FIXED_EE_Z_TOLERANCE = 0.05
ARM_PLANNING_TIME_S = 3.0
ARM_PLANNING_ATTEMPTS = 3
DESCENT_PLANNING_TIME_S = 5.0
ARM_POST_MOVE_SLEEP_S = 0.5
STARTUP_HOME_DELAY_S = 3.0
GRASP_STEP_TOTAL = 7
GRASP_STATUS_TOPIC = '/grasp/status'
GRASP_STATUS_IDLE = 'idle'
GRASP_STATUS_RUNNING = 'running'
# Legacy values; grasp cycle no longer publishes done/failed when arm is at place.
GRASP_STATUS_DONE = 'done'
GRASP_STATUS_FAILED = 'failed'
GRASP_STATUS_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
# Post-grasp lift: retry with slightly higher/lower z if start-height lift fails (m).
POST_GRASP_LIFT_RETRY_RAISE_OFFSETS = (0.012, 0.024, 0.036)
POST_GRASP_LIFT_RETRY_LOWER_OFFSETS = (-0.003, -0.006)
POST_GRASP_LIFT_RETRY_Z_OFFSETS = (
    (0.0,)
    + POST_GRASP_LIFT_RETRY_RAISE_OFFSETS
    + POST_GRASP_LIFT_RETRY_LOWER_OFFSETS
)
# After z-offset lift retries fail, nudge x/y slightly (m), up to 5 Pilz LIN attempts.
POST_GRASP_LIFT_XY_RETRY_MAX_ATTEMPTS = 5
POST_GRASP_LIFT_XY_RETRY_OFFSETS = (
    (0.003, 0.0),
    (-0.003, 0.0),
    (0.0, 0.003),
    (0.0, -0.003),
    (0.003, 0.003),
)
# Descent: Pilz LIN only; on failure raise target z slightly, up to 5 tries then OMPL direct.
DESCENT_LIN_MAX_ATTEMPTS = 5
DESCENT_LIN_RETRY_Z_OFFSETS = (0.0, 0.001, 0.002, 0.003, 0.004)
# Step 6: OMPL direct to place (fixed z).
# OMPL path: limit base-joint rotation from posture at plan start (rad, half-range).
BASE_JOINT_PATH_LIMITS = (
    ("panda_joint1", 0.50),
    ("panda_joint2", 0.40),
)
ARM_JOINT_NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))
_GRASP_TERMINATED = object()

# Gazebo spawn / table layout (worlds/arm_on_the_table.sdf + gazebo.launch.py)
ROBOT_WORLD_X = 0.05
ROBOT_WORLD_Y = 0.0
ROBOT_WORLD_Z = 1.02
TABLE_WORLD_X = 0.29
TABLE_WORLD_Y = 0.0
TABLE_WORLD_YAW = 1.5708
TABLE_SURFACE_Z_WORLD = 1.0
TABLE_SIZE_X = 1.5
TABLE_SIZE_Y = 0.8
TABLE_SIZE_Z = 0.03

# place_bin in worlds/arm_on_the_table.sdf — aligned with commander place point
PLACE_BIN_WORLD_X = ROBOT_WORLD_X + 0.3
PLACE_BIN_WORLD_Y = -0.3
PLACE_BIN_WORLD_Z = 1.002
PLACE_BIN_SIZE_X = 0.28
PLACE_BIN_SIZE_Y = 0.28
PLACE_BIN_SIZE_Z = 0.085

# worlds/arm_on_the_table.sdf — world name + movable object spawn poses (x,y,z,roll,pitch,yaw)
GZ_WORLD_NAME = "arm_and_table"
GZ_SET_POSE_SERVICE = f"/world/{GZ_WORLD_NAME}/set_pose"
# ros_gz_bridge SetEntityPose can hang after the first call on Gazebo Sim 6.
GZ_SET_POSE_ROS_TIMEOUT_SEC = 3.0
GZ_SET_POSE_INTERVAL_SEC = 0.05
from world_object_poses import WORLD_OBJECT_POSES  # noqa: E402


def _rpy_to_quaternion(roll, pitch, yaw):
    """SDF roll-pitch-yaw (rad) -> quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _transport_service_bin():
    """Headless Gazebo transport CLI (avoid Ruby ``ign`` launcher GUI popups)."""
    matches = sorted(glob.glob('/usr/libexec/gz/transport*/ign-transport-service'))
    if matches:
        return matches[-1]
    return shutil.which('ign-transport-service')

def _quaternion_multiply(q1, q2):
    """Hamilton product, (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def grasp_quaternion(yaw):
    """Top-down grasp in panda_link0: flip hand +Z toward -Z, then yaw about base Z."""
    q_flip_down = (1.0, 0.0, 0.0, 0.0)
    half = 0.5 * yaw
    q_yaw = (0.0, 0.0, math.sin(half), math.cos(half))
    return _quaternion_multiply(q_yaw, q_flip_down)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    plan_parameters=None,
    sleep_time=0.0,
):
    """Single plan + execute cycle; returns True on success. Sleeps sleep_time seconds after execution."""
    logger.info('Planning trajectory')
    if plan_parameters is not None:
        plan_result = planning_component.plan(parameters=plan_parameters)
    else:
        plan_result = planning_component.plan()
    if not plan_result:
        logger.error('Planning failed')
        return False

    logger.info('Executing plan')
    group_name = planning_component.planning_group_name
    status = robot.execute(group_name, plan_result.trajectory, blocking=True)
    logger.info(f'Execution status: {status}')
    time.sleep(sleep_time)
    return True


class Controller(Node):

    def __init__(self):
        super().__init__('commander')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
            self.listener_callback,
            10,
        )
        self.create_service(Trigger, '/reset_arm', self._handle_reset_arm)
        self.create_service(Trigger, '/reset_objects', self._handle_reset_objects)
        self._grasp_status_pub = self.create_publisher(
            String, GRASP_STATUS_TOPIC, GRASP_STATUS_QOS
        )
        self._set_pose_client = self.create_client(
            SetEntityPose, GZ_SET_POSE_SERVICE
        )

        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        self.panda = MoveItPy(node_name="moveit_cpp")
        self.arm_planning_component_name = "panda_arm"
        self.hand_planning_component_name = "panda_hand"

        self.panda_arm = self.panda.get_planning_component(self.arm_planning_component_name)
        self.panda_hand = self.panda.get_planning_component(self.hand_planning_component_name)
        self.logger = get_logger("moveit_py.pose_goal")

        self.arm_plan_params = PlanRequestParameters(self.panda)
        self.arm_plan_params.planning_pipeline = "ompl"
        self.arm_plan_params.planner_id = "RRTConnectConfigDefault"
        self.arm_plan_params.planning_time = ARM_PLANNING_TIME_S
        self.arm_plan_params.planning_attempts = ARM_PLANNING_ATTEMPTS

        self.arm_lin_plan_params = PlanRequestParameters(self.panda)
        self.arm_lin_plan_params.planning_pipeline = "pilz_industrial_motion_planner"
        self.arm_lin_plan_params.planner_id = "LIN"
        self.arm_lin_plan_params.planning_time = ARM_PLANNING_TIME_S
        self.arm_lin_plan_params.planning_attempts = ARM_PLANNING_ATTEMPTS
        self.arm_lin_plan_params.max_velocity_scaling_factor = 0.3
        self.arm_lin_plan_params.max_acceleration_scaling_factor = 0.3

        self.hand_plan_params = PlanRequestParameters(self.panda)
        self.hand_plan_params.planning_pipeline = "ompl"
        self.hand_plan_params.planner_id = "RRTConnectConfigDefault"
        self.hand_plan_params.planning_time = ARM_PLANNING_TIME_S
        self.hand_plan_params.planning_attempts = ARM_PLANNING_ATTEMPTS

        self._grasp_lock = threading.Lock()
        self._gripper_cmd_client = ActionClient(
            self, GripperCommand, GRIPPER_CMD_ACTION
        )
        self._gripper_goal_handle = None
        self._gripper_goal_lock = threading.Lock()
        self._finger_joint1_pos = GRIPPER_OPEN
        self._finger_joint2_pos = GRIPPER_OPEN
        self._arm_joint_positions = {name: 0.0 for name in ARM_JOINT_NAMES}

        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)

        self.height = 0.23
        self.pick_height = 0.119
        self.place_x = 0.3
        self.place_y = -0.3
        self.home_yaw = 0.0
        self.init_angle = -0.3825

        self._home_ready = threading.Event()
        self._startup_home_timer = self.create_timer(
            STARTUP_HOME_DELAY_S, self._on_startup_home_timer
        )

        self._setup_table_collision()
        self._setup_place_bin_collision()

    def _publish_grasp_status(self, status):
        msg = String()
        msg.data = status
        self._grasp_status_pub.publish(msg)
        self.get_logger().info(f'Grasp status -> {status}')

    def _on_joint_states(self, msg: JointState):
        for joint_name, attr in (
            (FINGER_JOINT1, "_finger_joint1_pos"),
            (FINGER_JOINT2, "_finger_joint2_pos"),
        ):
            try:
                idx = msg.name.index(joint_name)
                setattr(self, attr, float(msg.position[idx]))
            except (ValueError, IndexError):
                pass
        for joint_name in ARM_JOINT_NAMES:
            try:
                idx = msg.name.index(joint_name)
                self._arm_joint_positions[joint_name] = float(msg.position[idx])
            except (ValueError, IndexError):
                pass

    def _fingers_open_enough(self, target):
        tol = GRIPPER_OPEN_TOLERANCE
        return (
            self._finger_joint1_pos >= target - tol
            and self._finger_joint2_pos >= target - tol
        )

    def _setup_table_collision(self):
        """Add the Gazebo table to the MoveIt planning scene for OMPL/Pilz collision avoidance."""
        cx = TABLE_WORLD_X - ROBOT_WORLD_X
        cy = TABLE_WORLD_Y - ROBOT_WORLD_Y
        cz = TABLE_SURFACE_Z_WORLD - ROBOT_WORLD_Z
        half_yaw = 0.5 * TABLE_WORLD_YAW
        qz, qw = math.sin(half_yaw), math.cos(half_yaw)
        dim_x = TABLE_SIZE_Y
        dim_y = TABLE_SIZE_X
        dim_z = TABLE_SIZE_Z + 0.005

        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.id = "table_surface"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [dim_x, dim_y, dim_z]

        table_pose = Pose()
        table_pose.position.x = cx
        table_pose.position.y = cy
        table_pose.position.z = cz
        table_pose.orientation.z = qz
        table_pose.orientation.w = qw

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(table_pose)
        collision_object.operation = CollisionObject.ADD

        scene_monitor = self.panda.get_planning_scene_monitor()
        for attempt in range(10):
            try:
                with scene_monitor.read_write() as scene:
                    scene.apply_collision_object(collision_object)
                    scene.current_state.update()
                self.get_logger().info(
                    f"Table collision added in panda_link0 at "
                    f"({cx:.3f}, {cy:.3f}, {cz:.3f}), size=({dim_x}, {dim_y}, {dim_z})"
                )
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"Planning scene not ready ({attempt + 1}/10): {exc}"
                )
                time.sleep(0.5)
        self.get_logger().error("Failed to add table collision to planning scene")

    def _setup_place_bin_collision(self):
        """Add placement box collision near the arm place point."""
        cx = PLACE_BIN_WORLD_X - ROBOT_WORLD_X
        cy = PLACE_BIN_WORLD_Y - ROBOT_WORLD_Y
        cz = (
            PLACE_BIN_WORLD_Z - ROBOT_WORLD_Z + 0.5 * PLACE_BIN_SIZE_Z
        )

        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.id = "place_bin"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [PLACE_BIN_SIZE_X, PLACE_BIN_SIZE_Y, PLACE_BIN_SIZE_Z]

        bin_pose = Pose()
        bin_pose.position.x = cx
        bin_pose.position.y = cy
        bin_pose.position.z = cz
        bin_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(bin_pose)
        collision_object.operation = CollisionObject.ADD

        scene_monitor = self.panda.get_planning_scene_monitor()
        for attempt in range(10):
            try:
                with scene_monitor.read_write() as scene:
                    scene.apply_collision_object(collision_object)
                    scene.current_state.update()
                self.get_logger().info(
                    f"Place bin collision added in panda_link0 at "
                    f"({cx:.3f}, {cy:.3f}, {cz:.3f}), "
                    f"size=({PLACE_BIN_SIZE_X}, {PLACE_BIN_SIZE_Y}, "
                    f"{PLACE_BIN_SIZE_Z})"
                )
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"Place bin collision not ready ({attempt + 1}/10): {exc}"
                )
                time.sleep(0.5)
        self.get_logger().error(
            "Failed to add place bin collision to planning scene"
        )

    def _make_min_ee_axis_path_constraint(self, axis, min_value):
        """Keep EE on one side of a plane (x/y/z >= min_value) via a large bounding box."""
        half = 0.5 * MIN_EE_AXIS_BOX_SPAN
        constraints = Constraints()
        pos = PositionConstraint()
        pos.header.frame_id = "panda_link0"
        pos.link_name = EE_LINK

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            MIN_EE_AXIS_BOX_SPAN,
            MIN_EE_AXIS_BOX_SPAN,
            MIN_EE_AXIS_BOX_SPAN,
        ]

        region = BoundingVolume()
        region.primitives.append(box)
        region_pose = Pose()
        center = min_value + half
        if axis == "x":
            region_pose.position.x = center
        elif axis == "y":
            region_pose.position.y = center
        elif axis == "z":
            region_pose.position.z = center
        else:
            raise ValueError(f"unsupported EE axis constraint: {axis}")
        region.primitive_poses.append(region_pose)

        pos.constraint_region = region
        pos.weight = 1.0
        constraints.position_constraints.append(pos)
        return constraints

    def _make_min_ee_x_path_constraint(self, min_x=MIN_EE_X):
        return self._make_min_ee_axis_path_constraint("x", min_x)

    def _make_min_ee_z_path_constraint(self, min_z):
        return self._make_min_ee_axis_path_constraint("z", min_z)

    def _make_fixed_ee_z_path_constraint(self, target_z, tolerance=FIXED_EE_Z_TOLERANCE):
        constraints = Constraints()
        pos = PositionConstraint()
        pos.header.frame_id = "panda_link0"
        pos.link_name = EE_LINK

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [3.0, 3.0, 2.0 * tolerance]

        region = BoundingVolume()
        region.primitives.append(box)
        region_pose = Pose()
        region_pose.position.z = target_z
        region.primitive_poses.append(region_pose)

        pos.constraint_region = region
        pos.weight = 1.0
        constraints.position_constraints.append(pos)
        return constraints

    @staticmethod
    def _merge_path_constraints(*parts):
        merged = Constraints()
        for part in parts:
            if part is None:
                continue
            merged.joint_constraints.extend(part.joint_constraints)
            merged.position_constraints.extend(part.position_constraints)
            merged.orientation_constraints.extend(part.orientation_constraints)
            merged.visibility_constraints.extend(part.visibility_constraints)
        if (
            merged.joint_constraints
            or merged.position_constraints
            or merged.orientation_constraints
            or merged.visibility_constraints
        ):
            return merged
        return None

    def _make_base_joint_path_constraints(self):
        """Keep shoulder/base joints within a small band around current angles."""
        constraints = Constraints()
        for joint_name, half_range in BASE_JOINT_PATH_LIMITS:
            position = self._arm_joint_positions.get(joint_name, 0.0)
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = half_range
            joint_constraint.tolerance_below = half_range
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    def _gripper_goal_is_active_unlocked(self) -> bool:
        handle = self._gripper_goal_handle
        if handle is None:
            return False
        return handle.status in (
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        )

    def _cancel_gripper_goals(self):
        with self._gripper_goal_lock:
            self._gripper_goal_handle = None
        if not self._gripper_cmd_client.server_is_ready():
            return
        try:
            self._gripper_cmd_client._cancel_all_goals_async()
        except Exception as exc:
            self.get_logger().debug(f"Cancel gripper goals: {exc}")

    def _execute_gripper_command(
        self,
        position,
        max_effort,
        *,
        wait=True,
        allow_preempt=False,
        success_check=None,
    ):
        if not self._gripper_cmd_client.server_is_ready():
            if not self._gripper_cmd_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn("Gripper action server not ready")
                return False

        with self._gripper_goal_lock:
            if self._gripper_goal_is_active_unlocked() and not allow_preempt:
                if wait:
                    return self._wait_gripper_goal(self._gripper_goal_handle)
                return True

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        done_event = threading.Event()
        result = {"ok": False}

        def _on_result(future):
            try:
                action_result = future.result()
                status = action_result.status
                if success_check is not None:
                    result["ok"] = success_check(status)
                else:
                    result["ok"] = status == GoalStatus.STATUS_SUCCEEDED
                if not result["ok"]:
                    self.get_logger().warn(
                        f"Gripper command finished with status {status}"
                    )
            except Exception as exc:
                self.get_logger().warn(f"Gripper result error: {exc}")
            finally:
                done_event.set()

        def _on_goal_response(future):
            try:
                goal_handle = future.result()
                with self._gripper_goal_lock:
                    self._gripper_goal_handle = goal_handle
                if not goal_handle.accepted:
                    done_event.set()
                    return
                if wait:
                    goal_handle.get_result_async().add_done_callback(_on_result)
                else:
                    result["ok"] = True
                    done_event.set()
            except Exception as exc:
                self.get_logger().warn(f"Gripper goal rejected: {exc}")
                done_event.set()

        self._gripper_cmd_client.send_goal_async(goal).add_done_callback(
            _on_goal_response
        )

        if not done_event.wait(GRIPPER_CMD_TIMEOUT_S):
            self.get_logger().warn("Gripper command timed out")
            return False
        return result["ok"]

    def _wait_gripper_goal(self, goal_handle):
        done_event = threading.Event()
        result = {"ok": False}

        def _on_result(future):
            try:
                action_result = future.result()
                result["ok"] = action_result.status == GoalStatus.STATUS_SUCCEEDED
            except Exception as exc:
                self.get_logger().warn(f"Gripper wait error: {exc}")
            finally:
                done_event.set()

        goal_handle.get_result_async().add_done_callback(_on_result)
        if not done_event.wait(GRIPPER_CMD_TIMEOUT_S):
            self.get_logger().warn("Waiting for gripper goal timed out")
            return False
        return result["ok"]

    def _gripper_open_sync(self, position=GRIPPER_DESCENT_OPEN):
        ok = self._execute_gripper_command(
            position,
            GRIPPER_OPEN_MAX_EFFORT,
            wait=True,
            allow_preempt=True,
        )
        if not ok:
            self.get_logger().warn(
                "gripper_cmd open failed, falling back to MoveIt open"
            )
            ok = self.gripper_action("open")
        if ok:
            ok = self._wait_finger_open(position)
        return ok

    def _wait_finger_open(self, target, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._fingers_open_enough(target):
                self.get_logger().info(
                    f"gripper open reached: j1={self._finger_joint1_pos:.4f}, "
                    f"j2={self._finger_joint2_pos:.4f}, target={target:.4f}"
                )
                time.sleep(GRIPPER_OPEN_SETTLE_S)
                return True
            time.sleep(0.05)
        self.get_logger().warn(
            f"gripper open timeout: j1={self._finger_joint1_pos:.4f}, "
            f"j2={self._finger_joint2_pos:.4f}, target={target:.4f}"
        )
        return self._fingers_open_enough(target - GRIPPER_OPEN_TOLERANCE * 2)

    def _fingers_positions_stable(self, j1_ref, j2_ref, tolerance):
        return (
            abs(self._finger_joint1_pos - j1_ref) <= tolerance
            and abs(self._finger_joint2_pos - j2_ref) <= tolerance
        )

    def _wait_fingers_grasp_stable(self, timeout=None):
        if timeout is None:
            timeout = GRIPPER_GRASP_DETECT_TIMEOUT_S
        deadline = time.monotonic() + timeout
        stable_start = None
        stable_j1 = stable_j2 = None
        while time.monotonic() < deadline:
            if stable_start is None:
                stable_start = time.monotonic()
                stable_j1 = self._finger_joint1_pos
                stable_j2 = self._finger_joint2_pos
            elif not self._fingers_positions_stable(
                stable_j1, stable_j2, GRIPPER_GRASP_STABLE_TOLERANCE
            ):
                stable_start = time.monotonic()
                stable_j1 = self._finger_joint1_pos
                stable_j2 = self._finger_joint2_pos
            elif time.monotonic() - stable_start >= GRIPPER_GRASP_STABLE_S:
                self.get_logger().info(
                    f"gripper grasp stable: j1={self._finger_joint1_pos:.4f}, "
                    f"j2={self._finger_joint2_pos:.4f}, "
                    f"stable={GRIPPER_GRASP_STABLE_S:.1f}s"
                )
                return True
            time.sleep(0.05)
        self.get_logger().warn(
            f"gripper grasp detect timeout: j1={self._finger_joint1_pos:.4f}, "
            f"j2={self._finger_joint2_pos:.4f}"
        )
        return False

    def _gripper_close_sync(self):
        if not self._execute_gripper_command(
            GRIPPER_CLOSE,
            GRIPPER_OPEN_MAX_EFFORT,
            wait=False,
            allow_preempt=True,
        ):
            return False
        time.sleep(GRIPPER_CLOSE_AFTER_START_S)
        return self._wait_fingers_grasp_stable()

    def _standard_ompl_trials(self, x, y, z, yaw):
        """Step 1 OMPL: free -> min_ee_z -> relaxed fixed z (height may vary on early tries)."""
        return [
            (
                'OMPL free',
                lambda: self.move_to(
                    x, y, z, yaw, linear=False, apply_min_ee_x=False
                ),
            ),
            (
                f'OMPL min_ee_z>={CARRY_TRANSPORT_MIN_EE_Z}',
                lambda: self.move_to(
                    x,
                    y,
                    z,
                    yaw,
                    linear=False,
                    min_ee_z=CARRY_TRANSPORT_MIN_EE_Z,
                    apply_min_ee_x=False,
                ),
            ),
            (
                'OMPL relaxed fixed z',
                lambda: self.move_to(
                    x,
                    y,
                    z,
                    yaw,
                    linear=False,
                    fixed_ee_z=z,
                    fixed_ee_z_tolerance=0.05,
                    apply_min_ee_x=False,
                ),
            ),
        ]

    def _return_fixed_z_trials(self, x, y, z, yaw):
        """Return / transport: OMPL with EE z locked at goal height only."""
        tol = RETURN_FIXED_EE_Z_TOLERANCE
        return [
            (
                f'OMPL fixed z={z:.3f}±{tol:.3f}',
                lambda: self.move_to(
                    x,
                    y,
                    z,
                    yaw,
                    linear=False,
                    fixed_ee_z=z,
                    fixed_ee_z_tolerance=tol,
                    apply_min_ee_x=False,
                ),
            ),
        ]

    def _run_arm_motion_trials(self, step_label, trials):
        for label, attempt in trials:
            self.get_logger().info(f'{step_label} try: {label}')
            print(f'[Grasp] {step_label} try: {label}', flush=True)
            if attempt():
                return True
        return False

    def _move_above_target(self, x, y, yaw, motion_z):
        return self._run_arm_motion_trials(
            'Step 1',
            self._standard_ompl_trials(x, y, motion_z, yaw),
        )

    def _descend_to_pick(self, x, y, yaw):
        pick_z = self.pick_height
        offsets = DESCENT_LIN_RETRY_Z_OFFSETS[:DESCENT_LIN_MAX_ATTEMPTS]

        for offset in offsets:
            target_z = pick_z + offset
            if offset == 0.0:
                label = f'Pilz LIN z={target_z:.3f}'
            else:
                label = f'Pilz LIN z={target_z:.3f} (+{offset:.3f}m)'
            self.get_logger().info(f'Step 3 try: {label}')
            print(f'[Grasp] Step 3 try: {label}', flush=True)
            if self.move_to(x, y, target_z, yaw, linear=True, linear_only=True):
                return True

        def _descend_ompl_direct():
            saved_time = self.arm_plan_params.planning_time
            self.arm_plan_params.planning_time = DESCENT_PLANNING_TIME_S
            try:
                return self.move_to(x, y, pick_z, yaw, linear=False)
            finally:
                self.arm_plan_params.planning_time = saved_time

        label = f'OMPL direct z={pick_z:.3f}'
        self.get_logger().info(f'Step 3 try: {label}')
        print(f'[Grasp] Step 3 try: {label}', flush=True)
        return _descend_ompl_direct()

    def _return_to_place_with_fallback(self, yaw=None):
        """Startup/reset/failure return: OMPL free -> min_ee_z -> relaxed fixed z."""
        yaw = self.home_yaw if yaw is None else yaw
        return self._run_arm_motion_trials(
            'return to place',
            self._standard_ompl_trials(
                self.place_x, self.place_y, self.height, yaw
            ),
        )

    def _direct_return_to_place_and_finish_failed(self, failed_step):
        """OMPL back to place (gripper fixed during motion), open gripper at place, idle."""
        del failed_step
        if not self._return_to_place_with_fallback():
            self.get_logger().error('Abort: failed to move arm to place point')
            print('[Grasp] failed to plan return to place point', flush=True)
        else:
            self._gripper_open_sync(GRIPPER_OPEN)
        print('[Grasp] at home; waiting for next target', flush=True)
        self._publish_grasp_status(GRASP_STATUS_IDLE)

    def _abort_grasp_and_return_place(self, failed_step):
        self.get_logger().error(
            f'Grasp aborted at step {failed_step}, returning to place point'
        )
        print(
            f'[Grasp] step {failed_step} failed or timed out; aborting and returning to place '
            f'({self.place_x:.3f}, {self.place_y:.3f}, z={self.height:.3f})',
            flush=True,
        )
        self._direct_return_to_place_and_finish_failed(failed_step)

    def _try_pilz_lin_lift(self, x, y, z, yaw, label):
        self.get_logger().info(f'Step 5 try: {label}')
        print(f'[Grasp] Step 5 try: {label}', flush=True)
        return self.move_to(x, y, z, yaw, linear=True, linear_only=True)

    def _lift_after_grasp(self, x, y, yaw, motion_z):
        """Pilz LIN lift at pick; z/xy retries; else return to place and open gripper there."""
        lift_z = self.height if motion_z is None else motion_z
        for offset in POST_GRASP_LIFT_RETRY_Z_OFFSETS:
            target_z = lift_z + offset
            if offset == 0.0:
                label = f'Pilz LIN z={target_z:.3f}'
            elif offset > 0.0:
                label = f'Pilz LIN z={target_z:.3f} (+{offset:.3f}m)'
            else:
                label = f'Pilz LIN z={target_z:.3f} ({offset:.3f}m)'
            if self._try_pilz_lin_lift(x, y, target_z, yaw, label):
                return True

        xy_offsets = POST_GRASP_LIFT_XY_RETRY_OFFSETS[:POST_GRASP_LIFT_XY_RETRY_MAX_ATTEMPTS]
        for dx, dy in xy_offsets:
            target_x = x + dx
            target_y = y + dy
            label = (
                f'Pilz LIN ({target_x:.3f}, {target_y:.3f}, z={lift_z:.3f}) '
                f'(dx={dx:+.3f}, dy={dy:+.3f})'
            )
            if self._try_pilz_lin_lift(target_x, target_y, lift_z, yaw, label):
                return True

        self.get_logger().warn(
            'Step 5 lift failed at z-offset and xy-offset retries; '
            'falling back to direct return to place'
        )
        print(
            '[Grasp] step 5 lift failed; direct return to place, open gripper at place',
            flush=True,
        )
        self._direct_return_to_place_and_finish_failed(5)
        return _GRASP_TERMINATED

    def _transport_to_place(self, pick_x, pick_y, pick_yaw):
        """OMPL from pick hover to place: fixed z -> fixed z retry -> min_ee_z.

        The randomized planner occasionally fails on the fixed-z leg; retry the
        same leg once before relaxing to a min-ee-z path (object stays above
        CARRY_TRANSPORT_MIN_EE_Z but the height may vary).
        """
        del pick_x, pick_y, pick_yaw
        x = self.place_x
        y = self.place_y
        z = self.height
        yaw = self.home_yaw
        tol = RETURN_FIXED_EE_Z_TOLERANCE
        trials = self._return_fixed_z_trials(x, y, z, yaw)
        trials.append(
            (
                f'OMPL fixed z={z:.3f}±{tol:.3f} (retry)',
                lambda: self.move_to(
                    x,
                    y,
                    z,
                    yaw,
                    linear=False,
                    fixed_ee_z=z,
                    fixed_ee_z_tolerance=tol,
                    apply_min_ee_x=False,
                ),
            )
        )
        trials.append(
            (
                f'OMPL min_ee_z>={CARRY_TRANSPORT_MIN_EE_Z}',
                lambda: self.move_to(
                    x,
                    y,
                    z,
                    yaw,
                    linear=False,
                    min_ee_z=CARRY_TRANSPORT_MIN_EE_Z,
                    apply_min_ee_x=False,
                ),
            )
        )
        return self._run_arm_motion_trials('Step 6', trials)

    def move_to(
        self,
        x,
        y,
        z,
        yaw,
        linear=False,
        linear_only=False,
        min_ee_z=None,
        fixed_ee_z=None,
        fixed_ee_z_tolerance=None,
        limit_base_joints=False,
        apply_min_ee_x=True,
    ):
        qx, qy, qz, qw = grasp_quaternion(yaw)
        self.pose_goal.header.stamp = self.get_clock().now().to_msg()
        self.pose_goal.pose.position.x = x
        self.pose_goal.pose.position.y = y
        self.pose_goal.pose.position.z = z
        self.pose_goal.pose.orientation.x = qx
        self.pose_goal.pose.orientation.y = qy
        self.pose_goal.pose.orientation.z = qz
        self.pose_goal.pose.orientation.w = qw
        mode = "Pilz-LIN" if linear else "OMPL"
        self.logger.info(
            f"Goal [{mode}] {EE_LINK} in panda_link0: pos=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"yaw={yaw:.3f} rad, quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
        )

        def _plan_and_execute_arm(plan_params, label):
            ee_z_constraints = None
            if fixed_ee_z is not None:
                tol = (
                    FIXED_EE_Z_TOLERANCE
                    if fixed_ee_z_tolerance is None
                    else fixed_ee_z_tolerance
                )
                ee_z_constraints = self._make_fixed_ee_z_path_constraint(
                    fixed_ee_z, tolerance=tol
                )
            elif min_ee_z is not None:
                ee_z_constraints = self._make_min_ee_z_path_constraint(min_ee_z)
            base_constraints = (
                self._make_base_joint_path_constraints()
                if limit_base_joints
                else None
            )
            min_x_constraints = (
                self._make_min_ee_x_path_constraint(MIN_EE_X)
                if apply_min_ee_x
                else None
            )
            path_constraints = self._merge_path_constraints(
                min_x_constraints,
                base_constraints,
                ee_z_constraints,
            )
            if path_constraints is not None:
                self.panda_arm.set_path_constraints(path_constraints)
            try:
                self.panda_arm.set_start_state_to_current_state()
                self.panda_arm.set_goal_state(
                    pose_stamped_msg=self.pose_goal, pose_link=EE_LINK
                )
                self.logger.info(f"Planning arm trajectory ({label})")
                return plan_and_execute(
                    self.panda,
                    self.panda_arm,
                    self.logger,
                    plan_parameters=plan_params,
                    sleep_time=ARM_POST_MOVE_SLEEP_S,
                )
            finally:
                if path_constraints is not None:
                    self.panda_arm.set_path_constraints(Constraints())

        if linear:
            if _plan_and_execute_arm(self.arm_lin_plan_params, "Pilz-LIN"):
                return True
            if linear_only:
                return False
            self.logger.warn("Pilz LIN planning failed, falling back to OMPL")

        return _plan_and_execute_arm(self.arm_plan_params, "OMPL")

    def gripper_action(self, action):
        self.panda_hand.set_start_state_to_current_state()

        if action == "open":
            joint_values = {
                FINGER_JOINT1: GRIPPER_OPEN,
                FINGER_JOINT2: GRIPPER_OPEN,
            }
        elif action == "close":
            self.get_logger().warn(
                "gripper_action('close') deprecated; use _gripper_close_sync()"
            )
            return self._gripper_close_sync()
        else:
            self.get_logger().warn(f"Unknown gripper action: {action}")
            return False

        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.panda.get_robot_model().get_joint_model_group(
                self.hand_planning_component_name
            ),
        )        
        self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
        return plan_and_execute(
            self.panda,
            self.panda_hand,
            self.logger,
            plan_parameters=self.hand_plan_params,
            sleep_time=5.0,
        )

    def _log_grasp_step(self, step, message):
        line = f"[Grasp] step {step}/{GRASP_STEP_TOTAL}: {message}"
        print(line, flush=True)
        self.get_logger().info(line)

    def _log_grasp_step_done(self, step):
        line = f"[Grasp] step {step}/{GRASP_STEP_TOTAL} done"
        print(line, flush=True)
        self.get_logger().info(line)

    def _make_grasp_session(self, x, y, angle):
        yaw = self.init_angle + angle
        return {
            'x': x,
            'y': y,
            'angle': angle,
            'yaw': yaw,
            'motion_z': self.height,
        }

    def _grasp_step_description(self, step, session):
        x = session['x']
        y = session['y']
        yaw = session['yaw']
        z = session['motion_z']
        if step == 1:
            return f'move above target ({x:.3f}, {y:.3f}, z={z})'
        if step == 2:
            return f'open gripper at hover (z={z})'
        if step == 3:
            return (
                f'Pilz LIN descend to pick_height z={self.pick_height} '
                f'(retry +{DESCENT_LIN_RETRY_Z_OFFSETS[1]:.3f}m up to {DESCENT_LIN_MAX_ATTEMPTS}x, '
                f'then OMPL direct)'
            )
        if step == 4:
            return 'close gripper at pick height'
        if step == 5:
            return (
                f'Pilz LIN vertical lift at pick ({x:.3f}, {y:.3f}) to z≈{z} '
                f'(retry z +{POST_GRASP_LIFT_RETRY_RAISE_OFFSETS[0]:.3f}m / '
                f'{POST_GRASP_LIFT_RETRY_LOWER_OFFSETS[0]:.3f}m, then xy ±{POST_GRASP_LIFT_XY_RETRY_OFFSETS[0][0]:.3f}m x{POST_GRASP_LIFT_XY_RETRY_MAX_ATTEMPTS}, '
                f'then direct return)'
            )
        if step == 6:
            return (
                f'OMPL direct to place ({self.place_x}, {self.place_y}, z≈{z}, '
                f'fixed z±{RETURN_FIXED_EE_Z_TOLERANCE:.3f}; retry fixed z, '
                f'then min_ee_z>={CARRY_TRANSPORT_MIN_EE_Z})'
            )
        if step == 7:
            return 'open gripper to release'
        return f'step {step}'

    def _execute_grasp_step(self, step, session):
        x = session['x']
        y = session['y']
        yaw = session['yaw']
        motion_z = session['motion_z']

        if step == 1:
            return self._move_above_target(x, y, yaw, motion_z)
        if step == 2:
            return self._gripper_open_sync(GRIPPER_DESCENT_OPEN)
        if step == 3:
            return self._descend_to_pick(x, y, yaw)
        if step == 4:
            return self._gripper_close_sync()
        if step == 5:
            return self._lift_after_grasp(x, y, yaw, motion_z)
        if step == 6:
            return self._transport_to_place(x, y, yaw)
        if step == 7:
            return self._gripper_open_sync(GRIPPER_OPEN)
        return False

    def _on_startup_home_timer(self):
        self._startup_home_timer.cancel()
        worker = threading.Thread(
            target=self._startup_home_worker, daemon=True
        )
        worker.start()

    def _startup_home_worker(self):
        self._publish_grasp_status(GRASP_STATUS_RUNNING)
        try:
            self.get_logger().info(
                f"Startup homing to ({self.place_x:.3f}, {self.place_y:.3f}, "
                f"z={self.height:.3f}) above place point"
            )
            if not self._gripper_open_sync(GRIPPER_OPEN):
                self.get_logger().warn(
                    "Startup gripper open failed; continuing arm homing"
                )
            if self._return_to_place_with_fallback():
                self.get_logger().info("Startup homing succeeded")
            else:
                self.get_logger().error(
                    "Startup homing failed; check workspace / collisions"
                )
        except Exception as exc:
            self.get_logger().error(f"Startup homing error: {exc}")
        finally:
            self._home_ready.set()
            self._publish_grasp_status(GRASP_STATUS_IDLE)

    def _set_pose_via_ros(self, name, x, y, z, qx, qy, qz, qw):
        if not self._set_pose_client.service_is_ready():
            if not self._set_pose_client.wait_for_service(timeout_sec=1.0):
                return False
        request = SetEntityPose.Request()
        request.entity.name = name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        request.pose.orientation.x = float(qx)
        request.pose.orientation.y = float(qy)
        request.pose.orientation.z = float(qz)
        request.pose.orientation.w = float(qw)
        future = self._set_pose_client.call_async(request)
        deadline = time.monotonic() + GZ_SET_POSE_ROS_TIMEOUT_SEC
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f'SetEntityPose timed out for {name} after '
                    f'{GZ_SET_POSE_ROS_TIMEOUT_SEC:.1f}s'
                )
                return False
            time.sleep(0.02)
        if not future.done():
            return False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'SetEntityPose call failed for {name}: {exc}'
            )
            return False
        if not response.success:
            self.get_logger().error(f'SetEntityPose rejected for {name}')
        return response.success

    def _set_pose_via_transport_cli(self, name, x, y, z, qx, qy, qz, qw):
        transport_bin = _transport_service_bin()
        if transport_bin is None:
            self.get_logger().error(
                'set_pose failed: ign-transport-service not found'
            )
            return False
        req_text = (
            f'name: "{name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        env = os.environ.copy()
        env.pop('DISPLAY', None)
        result = subprocess.run(
            [
                transport_bin,
                '-s', GZ_SET_POSE_SERVICE,
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '3000',
                '-r', req_text,
            ],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
            env=env,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or '').strip()
            self.get_logger().error(
                f'transport set_pose failed for {name}: '
                f'{detail or result.returncode}'
            )
            return False
        return True

    def _gz_set_entity_pose(self, name, x, y, z, roll, pitch, yaw):
        qx, qy, qz, qw = _rpy_to_quaternion(roll, pitch, yaw)
        # Gazebo Sim 6: headless ign-transport-service is reliable; ROS bridge may hang.
        if _transport_service_bin() is not None:
            if self._set_pose_via_transport_cli(
                name, x, y, z, qx, qy, qz, qw
            ):
                return True
            self.get_logger().warn(
                f'transport set_pose failed for {name}; trying ROS'
            )
        if self._set_pose_via_ros(name, x, y, z, qx, qy, qz, qw):
            return True
        return False

    def _reset_world_objects(self):
        ok = True
        poses = WORLD_OBJECT_POSES
        for index, (name, x, y, z, roll, pitch, yaw) in enumerate(poses):
            if not self._gz_set_entity_pose(name, x, y, z, roll, pitch, yaw):
                ok = False
            elif index + 1 < len(poses):
                time.sleep(GZ_SET_POSE_INTERVAL_SEC)
        return ok

    def _handle_reset_arm(self, request, response):
        del request
        if not self._grasp_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Grasp in progress; try again later'
            return response
        try:
            self._publish_grasp_status(GRASP_STATUS_RUNNING)
            self.get_logger().info(
                f'Reset arm -> place ({self.place_x:.3f}, {self.place_y:.3f}, '
                f'z={self.height:.3f})'
            )
            print(
                f'[Reset] arm -> place point ({self.place_x:.3f}, {self.place_y:.3f}, '
                f'z={self.height:.3f})',
                flush=True,
            )
            if not self._gripper_open_sync(GRIPPER_OPEN):
                self.get_logger().warn('Reset arm: gripper open failed')
            arm_ok = self._return_to_place_with_fallback()
            response.success = arm_ok
            if arm_ok:
                response.message = 'Arm returned to place point'
                print('[Reset] arm reset complete', flush=True)
                self._publish_grasp_status(GRASP_STATUS_IDLE)
            else:
                response.message = 'Arm planning failed'
                print('[Reset] arm reset failed (planning failed)', flush=True)
                self._publish_grasp_status(GRASP_STATUS_FAILED)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Reset arm error: {exc}')
            self._publish_grasp_status(GRASP_STATUS_FAILED)
        finally:
            self._grasp_lock.release()
        return response

    def _handle_reset_objects(self, request, response):
        del request
        self.get_logger().info('Resetting world objects to spawn poses')
        print('[Reset] resetting table objects...', flush=True)
        try:
            ok = self._reset_world_objects()
            response.success = ok
            if ok:
                response.message = f'Reset {len(WORLD_OBJECT_POSES)} objects'
                print('[Reset] object reset complete', flush=True)
            else:
                response.message = 'One or more gz set_pose calls failed'
                print('[Reset] object reset partially failed; see commander log', flush=True)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Reset objects error: {exc}')
        return response

    def _run_grasp_sequence(self, x, y, angle):
        session = self._make_grasp_session(x, y, angle)

        for step in range(1, GRASP_STEP_TOTAL + 1):
            self._log_grasp_step(step, self._grasp_step_description(step, session))
            result = self._execute_grasp_step(step, session)
            if result is _GRASP_TERMINATED:
                return
            if not result:
                self._abort_grasp_and_return_place(step)
                return
            self._log_grasp_step_done(step)

        print('[Grasp] sequence complete', flush=True)
        self._publish_grasp_status(GRASP_STATUS_IDLE)

    def _grasp_sequence_worker(self, x, y, angle):
        try:
            self._publish_grasp_status(GRASP_STATUS_RUNNING)
            self._run_grasp_sequence(x, y, angle)
        except Exception as exc:
            self.get_logger().error(f'Grasp sequence failed: {exc}')
            self._abort_grasp_and_return_place(0)
        finally:
            self._grasp_lock.release()

    def listener_callback(self, data):
        if len(data.data) < 3:
            self.get_logger().warn("target_point needs [x, y, angle]")
            return

        self.get_logger().info(f"{data}")

        if not self._home_ready.is_set():
            self.get_logger().warn(
                "Startup homing not finished yet, ignoring target"
            )
            return

        x, y, angle = data.data[0], data.data[1], data.data[2]

        if not self._grasp_lock.acquire(blocking=False):
            self.get_logger().warn(
                "Grasp already in progress, ignoring target"
            )
            return

        worker = threading.Thread(
            target=self._grasp_sequence_worker,
            args=(x, y, angle),
            daemon=True,
        )
        worker.start()


def main():
    rclpy.init(args=None)
    controller = None
    executor_thread = None
    try:
        controller = Controller()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(controller)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        rate = controller.create_rate(2)
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[commander] Fatal error: {exc}', flush=True)
        raise SystemExit(1) from exc
    finally:
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if executor_thread is not None:
            executor_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
