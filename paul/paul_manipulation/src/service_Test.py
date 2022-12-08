#!/usr/bin/env python

from paul_manipulation.srv import ElevationPosition, ElevationPositionResponse
from paul_manipulation.msg import Elevation

import rospy

def test():
  msg = 1
  rospy.wait_for_service('elevation_controller_vision')
  try:
    elevationServiceResponse = rospy.ServiceProxy('/elevation_controller_vision', ElevationPosition)
    elevation = Elevation()
    elevation.data = msg
    resp1 = elevationServiceResponse(elevation)
    print("Service answer: " )
    print(resp1)
    return resp1
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

if __name__ == "__main__":
    test()
