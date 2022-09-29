#!/usr/bin/env python
from re import T
from threading import Thread
import rospy
from std_msgs.msg import Float64
from paul_control.msg import Elevation
from sensor_msgs.msg import JointState

from paul_control.srv import ElevationPosition, ElevationPositionResponse

MINIMAL_POSITION = 0
MAXIMAL_POSITION = 900
RAD_TOLERANCE = 20


###COMMANDE A EFFECTUER POUR SIMULER UN CLIENT SUR LE TERMINAL 
"""
rosservice call /elevation_controller_vision "level:           
  data: 1" 
"""

class ElevationCommandVision:
	def __init__(self):
		self.sub = rospy.Subscriber("/joint_states", JointState, self.GetODriveActualPosition)
		self.pub = rospy.Publisher('/paul_elevation_controller/command', Float64, queue_size=1)
		self.service = rospy.Service('elevation_controller_vision', ElevationPosition, self.PositionExecution)
		self.lastPosition = -50
		print("Ready to receive position")

	def GetODriveActualPosition(self, msg):
		self.lastPosition = msg.position[0]

	def PositionExecution(self, req):
		print("Received this req: ", req)

		cmd = Float64()

		if(req.level.data == Elevation.BOTTOM):
			cmd.data = MINIMAL_POSITION
			
		elif(req.level.data == Elevation.MIDDLE):
			cmd.data = MAXIMAL_POSITION/2

		elif(req.level.data == Elevation.UP):
			cmd.data = MAXIMAL_POSITION
		
		else:
			return {"success":False, "ActualPosition":self.lastPosition}

		self.pub.publish(cmd)

		print("Difference: ", abs(self.lastPosition - cmd.data))

		i = 0
		while(abs(self.lastPosition - cmd.data) >= RAD_TOLERANCE):
			if (i >= 100):
				break
			rospy.sleep(0.1)
			i += 1
			print(i)
		
		if (i >= 100):
			return {"success":False, "ActualPosition":self.lastPosition}
		else:
			return {"success":True, "ActualPosition":self.lastPosition}


if __name__ == "__main__":
	rospy.init_node('elevation_controller_vision', anonymous=True)
	ElevationCommandVision()
	rospy.spin()
