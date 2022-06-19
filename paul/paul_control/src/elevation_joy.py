#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class ElevationCommandPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/paul_joint0_velocity_controller/command', Float64, queue_size=1)
        self.sub = rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        cmd = data.axes[1]

        msg = Float64()
        msg.data = cmd * 75

        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('elevation', anonymous=True)

    ElevationCommandPublisher()
    rospy.spin()
