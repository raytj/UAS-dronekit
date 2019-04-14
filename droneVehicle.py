from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Rangefinder
import time
import math
from pymavlink import mavutil
from datetime import datetime



class maxSonar(Rangefinder):
	def __init__(self, *args, max_distance, min_distance=None, orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, id=0):
		super(maxSonar, self).__init__(*args)
		self.min_distance = min_distance
		self.max_distance = max_distance
		self.orientation = orientation
		self.id = id
		self.time_boost_ms = 0
		self. type = 1
		self.id = 0

class droneVehicle(Vehicle):
	def __init__(self, *args):
		super(droneVehicle, *args).__init__(*args)

	def send_SONAR_message(self, sonar, dist):
		self.msg = self.vehicle.message_factory.distance_sensor_encode(
			0,      # time system boot, not used
			sonar.min_distance,      # min disance cm
			sonar.max_distance,    # max dist cm
			dist,   # current dist, int cm
			0,      # type sensor
			sonar.id,      # on board id, not used
			sonar.orientation, # orientation: 0...7
			0,      # covariance, not used
			)
		self.vehicle.send_mavlink(self.msg)
