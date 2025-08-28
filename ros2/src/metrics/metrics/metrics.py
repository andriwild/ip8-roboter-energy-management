#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
import time


START_POSE = {
    'x': 15.2, 'y': 2.4, 'qx': 0.0, 'qy': 0.0, 'qz': -0.8, 'qw': 0.5 
}

TOUR_A_POSES = [
    {'x': 10.4, 'y': 4.36, 'qx': 0.0, 'qy': 0.0, 'qz': 0.97, 'qw': 0.23},
    {'x': 15.2, 'y': 2.4, 'qx': 0.0, 'qy': 0.0, 'qz': -0.8, 'qw': 0.5} ,
    {'x': 8.8, 'y': 3.5, 'qx': 0.0, 'qy': 0.0, 'qz': -0.16, 'qw': 1.0},
    {'x': 19.0, 'y': -3.1, 'qx': 0.0, 'qy': 0.0, 'qz': -0.52, 'qw': 0.83},
]

TOUR_B_POSES = [
    {'x': 32.2, 'y': -7.91, 'qx': 0.0, 'qy': 0.0, 'qz': 0.5, 'qw': 0.86},
    {'x': 4.33, 'y': 1.23, 'qx': 0.0, 'qy': 0.0, 'qz': 0.9, 'qw': 0.38},
]

class WaitBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, wait_time=3.0):
        super().__init__(name)
        self.wait_time = wait_time
        self.start_time = None
        
    def initialise(self):
        self.start_time = time.time()
        
    def update(self):
        if time.time() - self.start_time > self.wait_time:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class NavigationTree(Node):
    def __init__(self):
        super().__init__('navigation_tree')
        self.tree = self.create_tree()
        
    def create_pose(self, pose_dict):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_dict['x']
        pose.pose.position.y = pose_dict['y']
        pose.pose.position.z = pose_dict['qz']
        pose.pose.orientation.w = pose_dict['qw']
        return pose
        
    def create_tree(self):
        single_goal = NavigateToPose.Goal()
        single_goal.pose = self.create_pose(START_POSE)
        
        multi_goal = NavigateThroughPoses.Goal()
        multi_goal.poses = [
            self.create_pose(TOUR_A_POSES[0]),
            self.create_pose(TOUR_A_POSES[1]),
            #self.create_pose(TOUR_A_POSES[2]),
        ]
        
        navigate_single = py_trees_ros.action_clients.FromConstant(
            name="NavigateToSinglePose",
            action_type=NavigateToPose,
            action_name="/do150_0007/navigate_to_pose",
            action_goal=single_goal,
            #generate_feedback_message=lambda msg: self.get_logger().info(f"Navigating to single pose...")
        )
        
        wait_node = WaitBehavior(
            name="Wait3Seconds",
            wait_time=3.0
        )
        
        navigate_multi = py_trees_ros.action_clients.FromConstant(
            name="NavigateThroughMultiplePoses",
            action_type=NavigateThroughPoses,
            action_name="/do150_0007/navigate_through_poses",
            action_goal=multi_goal,
            #generate_feedback_message=lambda msg: self.get_logger().info(f"Navigating through poses, waypoint: {msg.current_pose}")
        )
        
        root = py_trees.composites.Sequence(
            name="MainSequence",
            memory=True,
            children=[
                #navigate_single,
                #wait_node,
                navigate_multi
            ]
        )

        root = py_trees.decorators.OneShot(
            name="RunOnce",
            child=root,
            policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        )
        
        tree = py_trees_ros.trees.BehaviourTree(
            root=root,
            unicode_tree_debug=True
        )
        
        tree.setup(timeout=15.0, node=self)
        return tree
    
    def spin(self):
        self.tree.tick_tock(period_ms=100)

def main():
    rclpy.init()
    nav_tree = NavigationTree()
    
    try:
        nav_tree.spin()
        rclpy.spin(nav_tree)
    except KeyboardInterrupt:
        pass
    finally:
        nav_tree.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

