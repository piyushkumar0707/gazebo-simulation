#!/usr/bin/env python3
"""
Simple ROS node to control the 3-DOF robot arm joints.
Publishes position commands to move the arm.
"""

import rospy
from std_msgs.msg import Float64
import math
import sys

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        
        # Publishers for each joint
        self.base_pub = rospy.Publisher(
            '/simple_3dof_arm/base_position_controller/command', 
            Float64, queue_size=10)
        self.shoulder_pub = rospy.Publisher(
            '/simple_3dof_arm/shoulder_position_controller/command', 
            Float64, queue_size=10)
        self.elbow_pub = rospy.Publisher(
            '/simple_3dof_arm/elbow_position_controller/command', 
            Float64, queue_size=10)
        
        rospy.sleep(1)  # Wait for publishers to connect
        
    def move_to_position(self, base_angle, shoulder_angle, elbow_angle):
        """Move all joints to specified angles (in radians)"""
        rospy.loginfo(f"Moving to: base={base_angle:.2f}, shoulder={shoulder_angle:.2f}, elbow={elbow_angle:.2f}")
        self.base_pub.publish(Float64(base_angle))
        self.shoulder_pub.publish(Float64(shoulder_angle))
        self.elbow_pub.publish(Float64(elbow_angle))
    
    def home_position(self):
        """Move to home position (all joints at 0)"""
        rospy.loginfo("Moving to home position...")
        self.move_to_position(0.0, 0.0, 0.0)
    
    def demo_sequence(self):
        """Run a demonstration sequence"""
        rate = rospy.Rate(0.5)  # 0.5 Hz = one move every 2 seconds
        
        rospy.loginfo("Starting demo sequence...")
        
        # Home position
        self.home_position()
        rate.sleep()
        
        # Rotate base
        rospy.loginfo("Rotating base...")
        self.move_to_position(1.57, 0.0, 0.0)  # 90 degrees
        rate.sleep()
        
        # Bend shoulder
        rospy.loginfo("Bending shoulder...")
        self.move_to_position(1.57, 0.7, 0.0)
        rate.sleep()
        
        # Bend elbow
        rospy.loginfo("Bending elbow...")
        self.move_to_position(1.57, 0.7, -0.7)
        rate.sleep()
        
        # Reach forward
        rospy.loginfo("Reaching forward...")
        self.move_to_position(0.0, 1.0, -1.2)
        rate.sleep()
        
        # Return home
        rospy.loginfo("Returning home...")
        self.home_position()
        rate.sleep()
        
        rospy.loginfo("Demo complete!")

def main():
    try:
        controller = ArmController()
        
        if len(sys.argv) > 1 and sys.argv[1] == 'demo':
            # Run demo sequence
            controller.demo_sequence()
        else:
            # Interactive mode
            rospy.loginfo("Arm controller ready!")
            rospy.loginfo("Usage examples:")
            rospy.loginfo("  Run demo: rosrun simple_3dof_arm control_arm.py demo")
            rospy.loginfo("  Or publish directly:")
            rospy.loginfo("  rostopic pub /simple_3dof_arm/base_position_controller/command std_msgs/Float64 'data: 1.57'")
            rospy.spin()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
