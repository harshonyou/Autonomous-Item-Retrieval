#!/usr/bin/env python3
'''
This Python script defines a NavigatorApp class that creates a GUI interface using the tkinter library.
 The GUI displays a single button labeled "Go Waiter",
which when pressed, navigates the robot to a predefined location (x=-1.69, y=4.9) using the nav2_simple_commander library.
'''
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import tkinter as tk

class NavigatorApp:
    def __init__(self):
        self.navigator=BasicNavigator()
        self.set_initial_pose()

    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 0.99
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def exiting(self):
        self.navigator.lifecycleShutdown()




def start_app():
    rclpy.init()
    app = NavigatorApp()
    app.exiting()
    rclpy.shutdown()