#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from roarm_msgs.srv import MoveToXYZ

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    RobotState,
)

from control_msgs.action import GripperCommand

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import threading
import math
import time


class MoveToXYZServer(Node):

    def __init__(self):

        super().__init__('xyz_control')

        # ============================================================
        # Existing variables
        # ============================================================

        self.last_xyz = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }

        self.x = None
        self.y = None
        self.z = None

        self.last_x = None
        self.last_y = None
        self.last_z = None

        self.executing = False
        self.detected = False
        self.full_stop = False
        self.counter = 0

        # ============================================================
        # Current robot joint state
        #
        # NEW:
        # Used to choose the IK solution closest to the current
        # physical configuration of the arm.
        # ============================================================

        self.current_joint_state = None

        # ============================================================
        # Callback group
        # ============================================================

        self.cb_group = ReentrantCallbackGroup()

        # ============================================================
        # Existing subscriptions
        # ============================================================

        self.subscriber_object_coords = self.create_subscription(
            Point,
            "object_coords",
            self.object_coords_callback,
            1
        )

        self.subscriber_det = self.create_subscription(
            Bool,
            "detection",
            self.detection_callback,
            1
        )

        self.subscriber_stop = self.create_subscription(
            Bool,
            "stop",
            self.stop_callback,
            1
        )

        # ============================================================
        # NEW: Joint state subscription
        # ============================================================

        self.subscriber_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        # ============================================================
        # Existing publisher
        # ============================================================

        self.publisher_exec = self.create_publisher(
            Bool,
            "executing_xyz",
            5
        )

        self.create_timer(
            0.5,
            self.control_loop
        )

        self.create_timer(
            0.5,
            self.publisher_exec_timer
        )

        # ============================================================
        # MoveGroup action client
        # ============================================================

        self.move_action_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.cb_group
        )

        self.get_logger().info(
            'Waiting for MoveGroup action server...'
        )

        self.move_action_client.wait_for_server()

        self.get_logger().info(
            'Connected to move_group'
        )

        # ============================================================
        # Gripper action client
        # ============================================================

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            callback_group=self.cb_group
        )

        self.get_logger().info(
            'Waiting for GripperCommand action server...'
        )

        self.gripper_client.wait_for_server()

        self.get_logger().info(
            'Connected to GripperCommand'
        )

        # ============================================================
        # IK service client
        # ============================================================

        self.ik_client = self.create_client(
            GetPositionIK,
            'compute_ik',
            callback_group=self.cb_group
        )

        # ============================================================
        # Pick service
        # ============================================================

        self.pick_srv = self.create_service(
            MoveToXYZ,
            'pick_sequential',
            self.handle_pick_sequential,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            'Service /pick_sequential ready'
        )

    # =================================================================
    # CALLBACKS
    # =================================================================

    def object_coords_callback(self, msg: Point):

        self.x = msg.x - 0.05
        self.y = -msg.y
        self.z = msg.z - 0.12

        print(self.x, self.y, self.z)

    def detection_callback(self, msg: Bool):

        self.detected = bool(msg.data)

    def stop_callback(self, msg):

        self.full_stop = msg.data


    def joint_state_callback(self, msg: JointState):

        # Store the latest joint state.
        #
        # Create a new JointState instead of directly keeping the
        # incoming message so that we have our own copy.
        arm_joint_names = {
            'base_link_to_link1',
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_gripper_link',
        }

        filtered_state = JointState()
        filtered_state.header = msg.header

        for index, joint_name in enumerate(msg.name):

            if joint_name not in arm_joint_names:
                continue

            if index >= len(msg.position):
                continue

            filtered_state.name.append(joint_name)
            filtered_state.position.append(msg.position[index])

            if index < len(msg.velocity):
                filtered_state.velocity.append(msg.velocity[index])

            if index < len(msg.effort):
                filtered_state.effort.append(msg.effort[index])

#        if not filtered_state.name:
#            self.get_logger().warning(
#                'No RoArm M3 joints found in /joint_states. '
#                'Cannot construct the arm IK seed.'
#            )
#            self.current_joint_state = None
#            return

        self.current_joint_state = filtered_state
#        self.current_joint_state = JointState()

#        self.current_joint_state.header = msg.header

#        self.current_joint_state.name = list(msg.name)
#        self.current_joint_state.position = list(msg.position)
#        self.current_joint_state.velocity = list(msg.velocity)
#        self.current_joint_state.effort = list(msg.effort)

    # =================================================================
    # EXECUTION STATUS
    # =================================================================

    def publisher_exec_timer(self):

        msg_exec = Bool()

        msg_exec.data = self.executing

        self.publisher_exec.publish(msg_exec)

    # =================================================================
    # CONTROL LOOP
    # =================================================================

    def control_loop(self):

        if self.full_stop:
            self.counter += 1

        if not self.full_stop:
            self.counter = 0

        if self.counter > 3 and self.detected:

            if (
                self.executing is True
                or self.x is None
                or self.y is None
                or self.z is None
            ):
                return

            self.executing = True

            req = MoveToXYZ.Request()

            if self.last_x is not None:

                if (
                    self.last_x - 0.01 <= self.x <= self.last_x + 0.01
                    and
                    self.last_y - 0.005 <= self.y <= self.last_y + 0.005
                    and
                    self.last_z - 0.002 <= self.z <= self.last_z + 0.002
                ):

                    self.executing = False

                    return

                if not self.detected:

                    self.x = self.last_x
                    self.y = self.last_y
                    self.z = self.last_z

                    self.executing = False

                    return

            req.x = self.x
            req.y = self.y
            req.z = self.z

            self.last_x = self.x
            self.last_y = self.y
            self.last_z = self.z

            service_call_obj = (
                self.create_client(
                    MoveToXYZ,
                    "pick_sequential"
                ).call_async(req)
            )

            service_call_obj.add_done_callback(
                self.done_callback
            )

        else:
            return

    # =================================================================
    # SERVICE CALLBACK
    # =================================================================

    def done_callback(self, service_call_obj):

        try:

            result = service_call_obj.result()

        except Exception as e:

            self.get_logger().error(
                f"{e}"
            )

        self.executing = False

    # =================================================================
    # GRIPPER
    # =================================================================

    def close_gripper(self):

        goal = GripperCommand.Goal()

        goal.command.position = 3.14
        goal.command.max_effort = 0.0

        event = threading.Event()

        future = self.gripper_client.send_goal_async(goal)

        future.add_done_callback(
            lambda f: event.set()
        )

        event.wait()

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Gripper goal rejected'
            )

            return False

        event = threading.Event()

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f: event.set()
        )

        event.wait()

        self.get_logger().info(
            'Gripper closed'
        )

        return True


    def open_gripper(self):

        goal = GripperCommand.Goal()

        goal.command.position = 1.5708
        goal.command.max_effort = 0.0

        event = threading.Event()

        future = self.gripper_client.send_goal_async(goal)

        future.add_done_callback(
            lambda f: event.set()
        )

        event.wait()

        goal_handle = future.result()

        if not goal_handle.accepted:

            return False

        event = threading.Event()

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f: event.set()
        )

        event.wait()

        self.get_logger().info(
            'Gripper opened'
        )

        return True

    # =================================================================
    # SEQUENTIAL PICK
    # =================================================================

    def handle_pick_sequential(self, request, response):

        # ------------------------------------------------------------
        # Open gripper
        # ------------------------------------------------------------

        self.open_gripper()

        obj_x = request.x
        obj_y = request.y
        obj_z = request.z

        self.get_logger().info(
            f'Sequential pick: '
            f'x={obj_x}, '
            f'y={obj_y}, '
            f'z={obj_z}'
        )

        # ------------------------------------------------------------
        # Calculate base angle
        # ------------------------------------------------------------

        base_angle = math.atan2(
            obj_y,
            obj_x
        )

        # ------------------------------------------------------------
        # Horizontal distance
        # ------------------------------------------------------------

        dist = math.sqrt(
            obj_x * obj_x
            +
            obj_y * obj_y
        )

        # ------------------------------------------------------------
        # STEP 1
        # Rotate / move toward object
        # ------------------------------------------------------------

        safe_dist = 0.06

        safe_x = (
            safe_dist
            *
            math.cos(base_angle)
        )

        safe_y = (
            safe_dist
            *
            math.sin(base_angle)
        )

        safe_z = 0.1

        self.get_logger().info(
            f'Step 1: Rotating to face object '
            f'(base={math.degrees(base_angle):.1f}°)'
        )

        step1_req = MoveToXYZ.Request()

        step1_req.x = safe_x
        step1_req.y = safe_y
        step1_req.z = safe_z

        step1_resp = MoveToXYZ.Response()

        self.handle_move_request(
            step1_req,
            step1_resp
        )

        time.sleep(0.4)

        if not step1_resp.success:

            response.success = False

            response.message = (
                'Step 1 failed: could not rotate'
            )

            return response

        # ------------------------------------------------------------
        # STEP 2
        # Move to object
        # ------------------------------------------------------------

        self.get_logger().info(
            f'Step 2: Moving to target '
            f'({obj_x}, {obj_y}, {obj_z})'
        )

        step2_req = MoveToXYZ.Request()

        step2_req.x = obj_x
        step2_req.y = obj_y
        step2_req.z = obj_z

        step2_resp = MoveToXYZ.Response()

        self.handle_move_request(
            step2_req,
            step2_resp
        )

        # Close gripper after reaching object.
        self.close_gripper()

        time.sleep(1)

        if not step2_resp.success:

            response.success = False

            response.message = (
                'Step 2 failed: could not reach target'
            )

            return response

        response.success = True

        # ------------------------------------------------------------
        # STEP 3
        # Move to trash drop
        # ------------------------------------------------------------

        self.get_logger().info(
            'Step 3: Moving to trash drop'
        )

        step3_req = MoveToXYZ.Request()

        step3_req.x = -0.20
        step3_req.y = 0.0
        step3_req.z = 0.1

        step3_resp = MoveToXYZ.Response()

        self.handle_move_request(
            step3_req,
            step3_resp
        )

        if not step3_resp.success:

            response.success = False

            response.message = (
                'Step 3 failed: could not drop'
            )

            return response

        # ------------------------------------------------------------
        # Release garbage
        # ------------------------------------------------------------

        self.open_gripper()

        time.sleep(0.5)

        # ------------------------------------------------------------
        # STEP 4
        # Return to initial position
        # ------------------------------------------------------------

        self.get_logger().info(
            'Step 4: Moving to init'
        )

        step4_req = MoveToXYZ.Request()

        step4_req.x = 0.1
        step4_req.y = 0.0
        step4_req.z = 0.15

        step4_resp = MoveToXYZ.Response()

        self.handle_move_request(
            step4_req,
            step4_resp
        )
        
        if not step4_resp.success:

            response.success = False

            response.message = (
                'Step 4 failed: could not go to init'
            )

            return response

        self.close_gripper()

        response.success = True

        response.message = (
            'Pick and drop sequence completed'
        )

        return response

    # =================================================================
    # IK SOLVER
    # =================================================================

    def solve_ik(self, x, y, z):

        pitch_values = [
            90,
            85,
            80,
            75,
            70,
            65,
            60,
            55,
            50,
            45,
            40,
            35,
            30,
            25,
            20,
            18,
            15,
            13,
            0,
            120,
            125,
            130,
            135,
            140,
            145,
            150,
            155,
            160,
            165,
            170,
            175,
            180
        ]

        base_angle = math.atan2(
            y,
            x
        )

        # Distance in horizontal plane.

        dist = math.sqrt(
            x * x
            +
            y * y
        )

        # The IK solver sees the arm in its own centered plane.

        link1_x = dist
        link1_y = 0.0
        link1_z = z

        self.get_logger().info(
            f'IK target: '
            f'x={x:.4f}, '
            f'y={y:.4f}, '
            f'z={z:.4f}, '
            f'base={math.degrees(base_angle):.1f}°, '
            f'dist={dist:.4f}'
        )

        # ============================================================
        # CURRENT JOINT STATE
        # ============================================================

        current_joint_state = self.current_joint_state

        if current_joint_state is None:

            self.get_logger().warn(
                'Current /joint_states not available yet. '
                'IK branch selection cannot use current position.'
            )

        else:

            self.get_logger().info(
                'Current joint state available for IK selection.'
            )

        # ============================================================
        # ANGULAR DISTANCE
        # ============================================================

        def angular_distance(a, b):

            return abs(
                math.atan2(
                    math.sin(a - b),
                    math.cos(a - b)
                )
            )

        # ============================================================
        # CALCULATE DISTANCE FROM CURRENT ARM CONFIGURATION
        # ============================================================

        def solution_distance(solution_joint_state):

            if current_joint_state is None:

                return float("inf")

            current_names = list(
                current_joint_state.name
            )

            current_positions = list(
                current_joint_state.position
            )

            solution_names = list(
                solution_joint_state.name
            )

            solution_positions = list(
                solution_joint_state.position
            )

            current_map = dict(
                zip(
                    current_names,
                    current_positions
                )
            )

            solution_map = dict(
                zip(
                    solution_names,
                    solution_positions
                )
            )

            total_distance = 0.0

            for joint_name, solution_position in solution_map.items():

                if joint_name not in current_map:

                    continue

                current_position = current_map[
                    joint_name
                ]

                difference = angular_distance(
                    solution_position,
                    current_position
                )

                total_distance += (
                    difference * difference
                )

            return math.sqrt(
                total_distance
            )
            
        valid_solutions = []

        last_result = None

        for pdeg in pitch_values:

            pitch = math.radians(
                pdeg
            )

            # --------------------------------------------------------
            # Quaternion
            # --------------------------------------------------------

            qw = math.cos(
                pitch / 2
            )

            qx = 0.0

            qy = math.sin(
                pitch / 2
            )

            qz = 0.0

            # --------------------------------------------------------
            # IK request
            # --------------------------------------------------------

            ik_request = GetPositionIK.Request()

            ik_request.ik_request.group_name = 'hand'

            ik_request.ik_request.pose_stamped = PoseStamped()

            ik_request.ik_request.pose_stamped.header.frame_id = (
                'world'
            )

            ik_request.ik_request.pose_stamped.pose.position = Point(
                x=link1_x,
                y=link1_y,
                z=link1_z
            )

            ik_request.ik_request.pose_stamped.pose.orientation = (
                Quaternion(
                    x=qx,
                    y=qy,
                    z=qz,
                    w=qw
                )
            )

            ik_request.ik_request.avoid_collisions = True

            # ========================================================
            # NEW:
            # Give MoveIt the current robot state as the IK seed.
            #
            # GetPositionIK expects a RobotState, not a JointState,
            # so we construct RobotState here.
            # ========================================================

            if current_joint_state is not None:

                robot_state = RobotState()

                robot_state.joint_state = (
                    current_joint_state
                )

                ik_request.ik_request.robot_state = (
                    robot_state
                )

            # --------------------------------------------------------
            # Call IK
            # --------------------------------------------------------

            self.get_logger().info(
                f'Trying IK pitch={pdeg}°'
            )

            event = threading.Event()

            future = self.ik_client.call_async(
                ik_request
            )

            future.add_done_callback(
                lambda f: event.set()
            )

            event.wait()

            # --------------------------------------------------------
            # Retrieve result
            # --------------------------------------------------------

            try:

                result = future.result()

            except Exception as e:

                self.get_logger().error(
                    f'IK service exception at '
                    f'pitch={pdeg}°: {e}'
                )

                continue

            last_result = result

            # --------------------------------------------------------
            # Check result
            # --------------------------------------------------------

            if result.error_code.val != 1:

                self.get_logger().warn(
                    f'FAILED: pitch={pdeg}°'
                )

                continue

            # ========================================================
            # VALID SOLUTION
            # ========================================================

            names = list(
                result.solution.joint_state.name
            )

            positions = list(
                result.solution.joint_state.position
            )

            # --------------------------------------------------------
            # Override base joint with your original base angle.
            # --------------------------------------------------------

            try:

                base_idx = names.index(
                    'base_link_to_link1'
                )

            except ValueError:

                self.get_logger().error(
                    "IK solution does not contain "
                    "'base_link_to_link1'"
                )

                continue

            positions[base_idx] = base_angle

            result.solution.joint_state.position = (
                positions
            )

            # --------------------------------------------------------
            # Calculate distance from current configuration.
            # --------------------------------------------------------

            distance = solution_distance(
                result.solution.joint_state
            )

            self.get_logger().info(
                f'IK SUCCESS: '
                f'pitch={pdeg}° '
                f'base={math.degrees(base_angle):.1f}° '
                f'joint_distance={distance:.5f}'
            )

            # --------------------------------------------------------
            # Store candidate.
            # --------------------------------------------------------

            valid_solutions.append(
                (
                    distance,
                    pdeg,
                    result
                )
            )

        # ============================================================
        # NO VALID SOLUTIONS
        # ============================================================

        if not valid_solutions:

            self.get_logger().error(
                f'No valid IK solution found for '
                f'({x:.4f}, {y:.4f}, {z:.4f})'
            )

            return last_result

        # ============================================================
        # SELECT BEST SOLUTION
        # ============================================================

        if current_joint_state is not None:

            # Smallest joint movement wins.

            valid_solutions.sort(
                key=lambda item: item[0]
            )

        else:

            # If we don't have joint state yet, preserve the
            # original behavior: first successful solution.

            valid_solutions.sort(
                key=lambda item: pitch_values.index(
                    item[1]
                )
            )

        best_distance = valid_solutions[0][0]
        best_pitch = valid_solutions[0][1]
        best_result = valid_solutions[0][2]

        # ============================================================
        # LOG SELECTED SOLUTION
        # ============================================================

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            f'SELECTED IK SOLUTION: '
            f'pitch={best_pitch}° '
            f'joint_distance={best_distance:.5f}'
        )

        self.get_logger().info(
            'Selected joints:'
        )

        selected_names = list(
            best_result.solution.joint_state.name
        )

        selected_positions = list(
            best_result.solution.joint_state.position
        )

        for joint_name, joint_position in zip(
            selected_names,
            selected_positions
        ):

            self.get_logger().info(
                f'  {joint_name}: '
                f'{joint_position:.5f}'
            )

        self.get_logger().info(
            '================================================'
        )

        return best_result

    # =================================================================
    # MOVE REQUEST
    # =================================================================

    def handle_move_request(
        self,
        request,
        response
    ):

        self.get_logger().info(
            f'Received: '
            f'x={request.x}, '
            f'y={request.y}, '
            f'z={request.z}'
        )

        # ============================================================
        # Solve IK
        # ============================================================

        ik_result = self.solve_ik(
            request.x,
            request.y,
            request.z
        )

        if ik_result is None:

            response.success = False

            response.message = (
                'IK returned no result'
            )

            self.get_logger().error(
                response.message
            )

            return response

        if ik_result.error_code.val != 1:

            response.success = False

            response.message = (
                f'IK failed with error code: '
                f'{ik_result.error_code.val}'
            )

            self.get_logger().error(
                response.message
            )

            return response

        # ============================================================
        # Extract IK solution
        # ============================================================

        joint_names = (
            ik_result.solution.joint_state.name
        )

        joint_positions = (
            ik_result.solution.joint_state.position
        )

        self.get_logger().info(
            f'IK solution: '
            f'{list(zip(joint_names, joint_positions))}'
        )

        # ============================================================
        # MoveGroup goal
        # ============================================================

        goal = MoveGroup.Goal()

        goal.request = MotionPlanRequest()

        goal.request.group_name = 'hand'

        goal.request.num_planning_attempts = 5

        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 1.0

        goal.request.max_acceleration_scaling_factor = 1.0

        # ============================================================
        # Joint constraints
        # ============================================================

        constraints = Constraints()

        for name, position in zip(
            joint_names,
            joint_positions
        ):

            # Keep original behavior:
            # gripper joint is not included in MoveGroup constraints.

            if name == 'link5_to_gripper_link':

                continue

            jc = JointConstraint()

            jc.joint_name = name

            jc.position = position

            jc.tolerance_above = 0.01

            jc.tolerance_below = 0.01

            jc.weight = 1.0

            constraints.joint_constraints.append(
                jc
            )

        goal.request.goal_constraints.append(
            constraints
        )

        # ============================================================
        # Planning options
        # ============================================================

        goal.planning_options.plan_only = False

        goal.planning_options.replan = True

        goal.planning_options.replan_attempts = 3

        # ============================================================
        # Send MoveGroup goal
        # ============================================================

        event = threading.Event()

        future = self.move_action_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            lambda f: event.set()
        )

        event.wait()

        goal_handle = future.result()

        if not goal_handle.accepted:

            response.success = False

            response.message = (
                'Goal rejected by move_group'
            )

            self.get_logger().error(
                response.message
            )

            return response

        self.get_logger().info(
            'Goal accepted, executing...'
        )

        # ============================================================
        # Wait for result
        # ============================================================

        event = threading.Event()

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            lambda f: event.set()
        )

        event.wait()

        result = (
            result_future.result().result
        )

        # ============================================================
        # Check result
        # ============================================================

        if result.error_code.val == 1:

            response.success = True

            response.message = (
                f'Moved to '
                f'({request.x}, '
                f'{request.y}, '
                f'{request.z})'
            )

            self.get_logger().info(
                response.message
            )

        else:

            response.success = False

            response.message = (
                f'Planning failed with error code: '
                f'{result.error_code.val}'
            )

            self.get_logger().error(
                response.message
            )

        return response


# =====================================================================
# MAIN
# =====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = MoveToXYZServer()

    executor = MultiThreadedExecutor()

    executor.add_node(
        node
    )

    executor.spin()

    node.destroy_node()

    rclpy.shutdown()


# =====================================================================
# ENTRY POINT
# =====================================================================

if __name__ == '__main__':

    main()
