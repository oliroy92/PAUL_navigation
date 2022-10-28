#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

import threading
import math

class ElevationCommandTester:
	def __init__(self):
		self.pub = rospy.Publisher('/paul_elevation_controller/command', Float64, queue_size=1)

	def test_loop(self):
		msg = Float64()

		top = 0
		mid = 90 * 2 * math.pi
		bot = 180 * 2 * math.pi

		sequence = [bot, top, mid, bot, mid, top]

		rospy.sleep(5)
		for i in range(30):
			print("Elevation Controller Test Iteration: ", i + 1)
			for item in sequence:
				msg.data = item
				self.pub.publish(msg)

				rospy.sleep(7)

if __name__ == '__main__':
	rospy.init_node('elevation_controller_test', anonymous=True)

	test = ElevationCommandTester()
	x = threading.Thread(target=test.test_loop, args=())
	x.start()

	rospy.spin()
