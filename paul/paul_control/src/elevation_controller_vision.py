#!/usr/bin/env python
from threading import Thread
import rospy
from std_msgs.msg import Float64
from paul_control.msg import Elevation
from sensor_msgs.msg import JointState

MINIMAL_POSITION = 0
MAXIMAL_POSITION = 900
RAD_TOLERANCE = 20

class ElevationCommandVision:
	def __init__(self):
		self.sub = rospy.Subscriber("/joint_states", JointState, self.GetODriveActualPosition)
		self.pub = rospy.Publisher('/paul_elevation_controller/command', Float64, queue_size=1)
		self.service = rospy.Service('elevation_controller_vision', self, self.PositionExecution)
		self.lastPosition = 0

	def GetODriveActualPosition(self, data):
		self.lastPosition = data.data
		

	def PositionExecution(self, req):
		print("Received this req: ", req)
		cmd = Float64()

		if(req.data == Elevation.BOTTOM):
			cmd.data = MINIMAL_POSITION
			
		elif(req.data == Elevation.MIDDLE):
			cmd.data = MAXIMAL_POSITION/2

		elif(req.data == Elevation.UP):
			cmd.data = MAXIMAL_POSITION
		
		else:
			return {"succes":False, "ActualPosition":self.lastPosition}

		self.pub.publish(cmd)

		i = 0
		while(abs(self.lastPosition - cmd.data) <= RAD_TOLERANCE or i >= 100):
			rospy.sleep(0.01)
			i += 1
			
		return {"succes":True, "ActualPosition":self.lastPosition}


if __name__ == '__main__':
	rospy.init_node('elevation_controller_vision', anonymous=True)

	t = ElevationCommandVision()
	rospy.spin()
