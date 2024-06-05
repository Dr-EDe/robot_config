#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('six_point_control')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('start')
        self.arm.go()
        rospy.sleep(2)

    def target_six_pose(self, pose_array):
        for i in range(len(pose_array)):
            self.arm.set_pose_target(pose_array[i], self.end_effector_link)
            self.arm.go()
            rospy.loginfo(
                "成功到达第{}个位置".format(i + 1) +
                "\ntarget_pose.pose.position.x = {:.5f}".format(pose_array[i].pose.position.x) +
                "\ntarget_pose.pose.position.y = {:.5f}".format(pose_array[i].pose.position.y) +
                "\ntarget_pose.pose.position.z = {:.5f}".format(pose_array[i].pose.position.z) +
                "\ntarget_pose.pose.orientation.x = {:.5f}".format(pose_array[i].pose.orientation.x) +
                "\ntarget_pose.pose.orientation.y = {:.5f}".format(pose_array[i].pose.orientation.y) +
                "\ntarget_pose.pose.orientation.z = {:.5f}".format(pose_array[i].pose.orientation.z) +
                "\ntarget_pose.pose.orientation.w = {:.5f}".format(pose_array[i].pose.orientation.w)
            )
            rospy.sleep(1)
        # 控制机械臂回到初始化位置
        self.arm.set_named_target('start')
        self.arm.go()
        rospy.loginfo("点位控制任务完成!")

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
        
        

if __name__ == "__main__":
    pose_array=[]
    test=MoveItIkDemo()
    target_pose1 = PoseStamped()
    target_pose1.header.frame_id = test.reference_frame
    target_pose1.header.stamp = rospy.Time.now()     
    target_pose1.pose.position.x = -0.20954
    target_pose1.pose.position.y = 0.089024
    target_pose1.pose.position.z = 0.60222
    target_pose1.pose.orientation.x = 0.37667
    target_pose1.pose.orientation.y = 0.75248
    target_pose1.pose.orientation.z = 0.4445
    target_pose1.pose.orientation.w = -0.30711
    
    target_pose2 = PoseStamped()
    target_pose2.header.frame_id = test.reference_frame
    target_pose2.header.stamp = rospy.Time.now()     
    target_pose2.pose.position.x = -0.1128
    target_pose2.pose.position.y =  -0.14707
    target_pose2.pose.position.z = 0.37064
    target_pose2.pose.orientation.x = -0.39081
    target_pose2.pose.orientation.y = 0.83395
    target_pose2.pose.orientation.z = -0.38933
    target_pose2.pose.orientation.w = -0.01492

    target_pose3 = PoseStamped()
    target_pose3.header.frame_id = test.reference_frame
    target_pose3.header.stamp = rospy.Time.now()     
    target_pose3.pose.position.x = 0.026495
    target_pose3.pose.position.y = -0.33704
    target_pose3.pose.position.z = 0.36236
    target_pose3.pose.orientation.x = -0.30971
    target_pose3.pose.orientation.y = -0.19614
    target_pose3.pose.orientation.z = -0.22526
    target_pose3.pose.orientation.w = 0.9027

    target_pose4 = PoseStamped()
    target_pose4.header.frame_id = test.reference_frame
    target_pose4.header.stamp = rospy.Time.now()     
    target_pose4.pose.position.x = -0.38729
    target_pose4.pose.position.y = -0.16604
    target_pose4.pose.position.z = 0.41109
    target_pose4.pose.orientation.x = 0.45951
    target_pose4.pose.orientation.y = -0.17333
    target_pose4.pose.orientation.z = 0.77484
    target_pose4.pose.orientation.w = -0.39803

    target_pose5 = PoseStamped()
    target_pose5.header.frame_id = test.reference_frame
    target_pose5.header.stamp = rospy.Time.now()     
    target_pose5.pose.position.x = 0.13853
    target_pose5.pose.position.y = 0.36664
    target_pose5.pose.position.z = 0.3988
    target_pose5.pose.orientation.x = -0.20134
    target_pose5.pose.orientation.y = -0.0048734
    target_pose5.pose.orientation.z = 0.58476
    target_pose5.pose.orientation.w = 0.78581

    target_pose6 = PoseStamped()
    target_pose6.header.frame_id = test.reference_frame
    target_pose6.header.stamp = rospy.Time.now()     
    target_pose6.pose.position.x = -0.42208
    target_pose6.pose.position.y = -0.010391
    target_pose6.pose.position.z = 0.37943
    target_pose6.pose.orientation.x = 0.39368
    target_pose6.pose.orientation.y = -0.1043
    target_pose6.pose.orientation.z = 0.91279
    target_pose6.pose.orientation.w = 0.031039 
    pose_array=[target_pose1,target_pose2,target_pose3,target_pose4,target_pose5,target_pose6]
    
    test.target_six_pose(pose_array)
    
    
