#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import time
import yaml
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# smach (ROS2)
from smach import State, StateMachine
import smach_ros

# navigation (Nav2)
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
import geometry_msgs.msg

# tf2 for quaternion
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation

# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# ament_index for package path
from ament_index_python.packages import get_package_share_directory

# hand control service
from seed_r7_ros_controller.srv import HandControl


# Global node and clients (initialized in main)
g_node = None
g_moveit = None


###########################################
class NaviAction:
    def __init__(self, node):
        self.node_ = node

        pkg_share = get_package_share_directory('seed_r7_samples')
        with open(pkg_share + '/config/waypoints.yaml') as f:
            self.config = yaml.safe_load(f)

        self._ac = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.node_.get_logger().info('Waiting for navigate_to_pose action server...')
        self._ac.wait_for_server()
        self.node_.get_logger().info('navigate_to_pose server is up')

    def set_goal(self, number):
        rev = dict(self.config[number])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node_.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = rev['pose']['position']['x']
        goal_msg.pose.pose.position.y = rev['pose']['position']['y']
        goal_msg.pose.pose.position.z = rev['pose']['position']['z']
        goal_msg.pose.pose.orientation.x = rev['pose']['orientation']['x']
        goal_msg.pose.pose.orientation.y = rev['pose']['orientation']['y']
        goal_msg.pose.pose.orientation.z = rev['pose']['orientation']['z']
        goal_msg.pose.pose.orientation.w = rev['pose']['orientation']['w']

        self.node_.get_logger().info('Sending navigation goal')
        future = self._ac.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node_, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.node_.get_logger().warn('Goal rejected')
            return 'aborted'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node_, result_future, timeout_sec=60.0)
        if result_future.done():
            self.node_.get_logger().info('Navigation succeeded')
            return 'succeeded'
        else:
            self.node_.get_logger().info('Navigation failed or timed out')
            return 'aborted'


#---------------------------------
class GO_TO_PLACE(State):
    def __init__(self, place):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.place_ = place

    def execute(self, userdata):
        g_node.get_logger().info('Going to Place{}'.format(self.place_))
        return na.set_goal(self.place_)


###########################################
class HandController:
    def __init__(self, node):
        self.node_ = node
        self.cli_ = node.create_client(HandControl, '/seed_r7_ros_controller/hand_control')
        node.get_logger().info('Waiting for hand_control service...')
        self.cli_.wait_for_service()

    def _call(self, request):
        future = self.cli_.call_async(request)
        rclpy.spin_until_future_complete(self.node_, future, timeout_sec=5.0)
        return future.done()

    def grasp(self):
        req = HandControl.Request()
        req.hand_id = 0
        req.command = 'grasp'
        req.power = 100
        return self._call(req)

    def release(self):
        req = HandControl.Request()
        req.hand_id = 0
        req.command = 'release'
        req.power = 100
        return self._call(req)


###########################################
class MoveitCommand:
    def __init__(self, node):
        self.node_ = node
        self.moveit_ = g_moveit
        self.robot_model = node.get_parameter_or(
            'robot_model_plugin', rclpy.Parameter('s', value='')).value

        self.box1 = self._box_pose(0.6, -0.3, 0.36)
        self.box2 = self._box_pose(0.7,  0.3, 0.76)
        self.box3 = self._box_pose(0.8,  0.0, 1.16)

        self.scene = self.moveit_.get_planning_scene_monitor()

    def _box_pose(self, x, y, z):
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        return ps

    def _quat_from_rpy(self, r, p, y):
        rot = Rotation.from_euler('xyz', [r, p, y])
        q = rot.as_quat()   # [x, y, z, w]
        qmsg = geometry_msgs.msg.Quaternion()
        qmsg.x, qmsg.y, qmsg.z, qmsg.w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return qmsg

    def set_grasp_position(self, x, y, z, vel=1.0, direction='side'):
        arm = self.moveit_.get_planning_component('rarm_with_torso')
        arm.set_start_state_to_current_state()

        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)

        if direction == 'side':
            eef_link = 'r_eef_grasp_link'
            target.pose.orientation = self._quat_from_rpy(0, 0, 0)
        else:  # top
            eef_link = 'r_eef_pick_link'
            target.pose.orientation = self._quat_from_rpy(-1.57, 0.79, 0)

        arm.set_goal_state(pose_stamped_msg=target, pose_link=eef_link)

        plan_result = arm.plan()
        if not plan_result:
            self.node_.get_logger().warn("IK can't be solved")
            return 'aborted'

        self.moveit_.execute(plan_result.trajectory, controllers=[])
        return 'succeeded'

    def set_lifter_position(self, x, z, vel=1.0):
        if 'typef' in self.robot_model:
            distance_body_lifter = 1.065 - 0.92
        else:
            distance_body_lifter = 0.994 - 0.857

        torso = self.moveit_.get_planning_component('torso')
        torso.set_start_state_to_current_state()

        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = float(x)
        target.pose.position.y = 0.0
        target.pose.position.z = float(z) + distance_body_lifter
        target.pose.orientation.w = 1.0

        torso.set_goal_state(pose_stamped_msg=target, pose_link='body_link')
        plan_result = torso.plan()
        if not plan_result:
            self.node_.get_logger().warn("can't be solved lifter ik")
            return 'aborted'

        self.moveit_.execute(plan_result.trajectory, controllers=[])
        return 'succeeded'

    def set_initial_pose(self):
        upper = self.moveit_.get_planning_component('upper_body')
        upper.set_start_state_to_current_state()
        upper.set_goal_state(configuration_name='reset-pose')
        plan_result = upper.plan()
        if not plan_result:
            self.node_.get_logger().warn("IK can't be solved")
            return 'aborted'
        self.moveit_.execute(plan_result.trajectory, controllers=[])
        return 'succeeded'

    def add_objects(self):
        with self.scene.read_write() as scene:
            scene.apply_collision_object(
                self._make_box('shelf1', 0.8, 0.0, 0.3,  0.5, 1.0, 0.01))
            scene.apply_collision_object(
                self._make_box('shelf2', 0.8, 0.0, 0.7,  0.5, 1.0, 0.01))
            scene.apply_collision_object(
                self._make_box('shelf3', 0.8, 0.0, 1.1,  0.5, 1.0, 0.01))
            scene.apply_collision_object(
                self._make_box('wall1',  0.8, 0.5, 0.75, 0.5, 0.01, 1.5))
            scene.apply_collision_object(
                self._make_box('wall2',  0.8,-0.5, 0.75, 0.5, 0.01, 1.5))
            scene.apply_collision_object(
                self._make_box('box1',
                               self.box1.pose.position.x,
                               self.box1.pose.position.y,
                               self.box1.pose.position.z, 0.05, 0.1, 0.1))
            scene.apply_collision_object(
                self._make_box('box2',
                               self.box2.pose.position.x,
                               self.box2.pose.position.y,
                               self.box2.pose.position.z, 0.05, 0.05, 0.1))
            scene.apply_collision_object(
                self._make_box('box3',
                               self.box3.pose.position.x,
                               self.box3.pose.position.y,
                               self.box3.pose.position.z, 0.05, 0.05, 0.1))
        return 'succeeded'

    def remove_objects(self):
        with self.scene.read_write() as scene:
            for name in ['shelf1', 'shelf2', 'shelf3', 'wall1', 'wall2',
                         'box1', 'box2', 'box3']:
                scene.remove_collision_object(name)
        return 'succeeded'

    def attach_objects(self, object_name):
        with self.scene.read_write() as scene:
            scene.attach_collision_object(object_name, 'r_eef_pick_link',
                                          ['r_hand_link', 'r_thumb_link',
                                           'r_indexbase_link'])
        return 'succeeded'

    def detach_objects(self, object_name):
        with self.scene.read_write() as scene:
            scene.detach_collision_object(object_name)
        return 'succeeded'

    @staticmethod
    def _make_box(name, x, y, z, sx, sy, sz):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = name
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [sx, sy, sz]
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj


#---------------------------------
class MANIPULATE(State):
    def __init__(self, x, y, z, vel=1.0, direction='side'):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.x, self.y, self.z = x, y, z
        self.vel = vel
        self.direction = direction

    def execute(self, userdata):
        g_node.get_logger().info(
            'Manipulate at ({},{},{})'.format(self.x, self.y, self.z))
        return mc.set_grasp_position(self.x, self.y, self.z,
                                     self.vel, self.direction)


class MOVE_LIFTER(State):
    def __init__(self, x, z, vel=1.0):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.x, self.z, self.vel = x, z, vel

    def execute(self, userdata):
        g_node.get_logger().info(
            'Move Lifter at ({},{})'.format(self.x, self.z))
        return mc.set_lifter_position(self.x, self.z, self.vel)


class INIT_POSE(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        g_node.get_logger().info('initialize wholebody')
        return mc.set_initial_pose()


class UPDATE_OBJECTS(State):
    def __init__(self, action):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.action = action

    def execute(self, userdata):
        if self.action == 'add':
            return mc.add_objects()
        else:
            return mc.remove_objects()


class PICK(State):
    def __init__(self, object_name):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.object_name = object_name

    def execute(self, userdata):
        if mc.attach_objects(self.object_name) == 'succeeded':
            return 'succeeded' if hc.grasp() else 'aborted'
        return 'aborted'


class PLACE(State):
    def __init__(self, object_name):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.object_name = object_name

    def execute(self, userdata):
        if mc.detach_objects(self.object_name) == 'succeeded':
            return 'succeeded' if hc.release() else 'aborted'
        return 'aborted'


#==================================
if __name__ == '__main__':
    rclpy.init()
    g_node = Node('scenario_node')

    # MoveItPy needs to be created with the same node
    g_moveit = MoveItPy(node_name='scenario_moveit_py')

    na = NaviAction(g_node)
    hc = HandController(g_node)
    mc = MoveitCommand(g_node)

    # Spin node in background
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(g_node,), daemon=True)
    spin_thread.start()

    # Build state machine
    go_to_shelf = StateMachine(outcomes=['succeeded', 'aborted'])
    with go_to_shelf:
        StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0, 0.6),
                         transitions={'succeeded': 'MOVE', 'aborted': 'aborted'})
        StateMachine.add('MOVE', GO_TO_PLACE(1),
                         transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    go_to_start_point = StateMachine(outcomes=['succeeded', 'aborted'])
    with go_to_start_point:
        StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0, 0.6),
                         transitions={'succeeded': 'MOVE', 'aborted': 'aborted'})
        StateMachine.add('MOVE', GO_TO_PLACE(0),
                         transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    x1, y1, z1 = (mc.box1.pose.position.x,
                  mc.box1.pose.position.y, mc.box1.pose.position.z)
    x2, y2, z2 = (mc.box2.pose.position.x,
                  mc.box2.pose.position.y, mc.box2.pose.position.z)
    x3, y3, z3 = (mc.box3.pose.position.x,
                  mc.box3.pose.position.y, mc.box3.pose.position.z)

    pick_place_1 = StateMachine(outcomes=['succeeded', 'aborted'])
    with pick_place_1:
        StateMachine.add('PICK MOTION', MANIPULATE(x1, y1, z1, direction='top'),
                         transitions={'succeeded': 'PICK', 'aborted': 'aborted'})
        StateMachine.add('PICK', PICK('box1'),
                         transitions={'succeeded': 'PLACE MOTION', 'aborted': 'aborted'})
        StateMachine.add('PLACE MOTION', MANIPULATE(0.6, -0.4, z1 + 0.4, direction='top'),
                         transitions={'succeeded': 'PLACE', 'aborted': 'aborted'})
        StateMachine.add('PLACE', PLACE('box1'),
                         transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    pick_place_2 = StateMachine(outcomes=['succeeded', 'aborted'])
    with pick_place_2:
        StateMachine.add('PICK MOTION', MANIPULATE(x2, y2, z2),
                         transitions={'succeeded': 'PICK', 'aborted': 'aborted'})
        StateMachine.add('PICK', PICK('box2'),
                         transitions={'succeeded': 'PLACE MOTION', 'aborted': 'aborted'})
        StateMachine.add('PLACE MOTION', MANIPULATE(0.6, -0.2, z2),
                         transitions={'succeeded': 'PLACE', 'aborted': 'aborted'})
        StateMachine.add('PLACE', PLACE('box2'),
                         transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    pick_place_3 = StateMachine(outcomes=['succeeded', 'aborted'])
    with pick_place_3:
        StateMachine.add('PICK MOTION', MANIPULATE(x3, y3, z3),
                         transitions={'succeeded': 'PICK', 'aborted': 'aborted'})
        StateMachine.add('PICK', PICK('box3'),
                         transitions={'succeeded': 'PLACE MOTION', 'aborted': 'aborted'})
        StateMachine.add('PLACE MOTION', MANIPULATE(0.6, 0.0, z3 - 0.4),
                         transitions={'succeeded': 'PLACE', 'aborted': 'aborted'})
        StateMachine.add('PLACE', PLACE('box3'),
                         transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    scenario_play = StateMachine(outcomes=['succeeded', 'aborted'])
    with scenario_play:
        StateMachine.add('GO TO SHELF', go_to_shelf,
                         transitions={'succeeded': 'ADD OBJECTS', 'aborted': 'aborted'})
        StateMachine.add('ADD OBJECTS', UPDATE_OBJECTS('add'),
                         transitions={'succeeded': 'PICK and PLACE 1', 'aborted': 'aborted'})
        StateMachine.add('PICK and PLACE 1', pick_place_1,
                         transitions={'succeeded': 'PICK and PLACE 2', 'aborted': 'aborted'})
        StateMachine.add('PICK and PLACE 2', pick_place_2,
                         transitions={'succeeded': 'PICK and PLACE 3', 'aborted': 'aborted'})
        StateMachine.add('PICK and PLACE 3', pick_place_3,
                         transitions={'succeeded': 'INITIALIZE', 'aborted': 'aborted'})
        StateMachine.add('INITIALIZE', INIT_POSE(),
                         transitions={'succeeded': 'REMOVE OBJECTS', 'aborted': 'aborted'})
        StateMachine.add('REMOVE OBJECTS', UPDATE_OBJECTS('remove'),
                         transitions={'succeeded': 'GO TO START POINT', 'aborted': 'aborted'})
        StateMachine.add('GO TO START POINT', go_to_start_point,
                         transitions={'succeeded': 'GO TO SHELF', 'aborted': 'aborted'})

    sis = smach_ros.IntrospectionServer('server_name', scenario_play,
                                        '/SEED-Noid-Mover Scenario Play')
    sis.start()
    scenario_play.execute()
    sis.stop()

    g_node.destroy_node()
    rclpy.shutdown()
