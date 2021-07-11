#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoystick():
    def __init__(self):
        rospy.init_node("teleop_joystick", anonymous = True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCB)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        #Getting parameters initialized at the launch file
        self.enable_button = rospy.get_param("~enable_button")
        self.linear_scaling = rospy.get_param("~scale_linear")
        self.angular_scaling = rospy.get_param("~scale_angular")
        self.axis_linear = rospy.get_param("~axis_linear")
        self.axis_angular = rospy.get_param("~axis_angular")
    
    def limitSpeed(self, input_speed):

        output_speed = 0.0

        if input_speed >= 0.2:
            output_speed = 0.2
                    
        elif input_speed <= -0.2:
            output_speed = -0.2
            
        else:
            output_speed = 0.0
    
        return output_speed

    def joyCB(self, data): #Publishes the vehicle movements commands to the cmd_vel topic

        twist = Twist()
        twist.linear.x = data.axes[self.axis_linear]
        twist.angular.z = data.axes[self.axis_angular]

        if data.buttons[self.enable_button] == 1: #Verify the dead man button is active (RB)
            twist.angular.z = self.limitSpeed(twist.angular.z) * self.angular_scaling
            twist.linear.x = self.limitSpeed(twist.linear.x) * self.linear_scaling
            self.cmd_pub.publish(twist)

        else:
            twist.angular.z = 0
            twist.linear.x = 0
            self.cmd_pub.publish(twist)

if __name__ == '__main__':
    teleop_joystick = TeleopJoystick()
    rospy.loginfo("teleop_joystick ready")
    rospy.spin()

