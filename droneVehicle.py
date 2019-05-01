from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Rangefinder, Vehicle
import time
import math
from pymavlink import mavutil
from datetime import datetime



class maxSonar(object):
    def __init__(self, max_distance=0, min_distance=0, orientation=0, id=0):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.orientation = orientation
        self.id = id
        self.time_boost_ms = 0
        self. type = 10
        self.distance = min_distance
        self.raw_voltage = 0
    def update(self, distance, raw_voltage=0):
        self.distance = distance
        self.raw_voltage = raw_voltage

class droneVehicle(Vehicle):
    def __init__(self, *args):
        super(droneVehicle, self).__init__(*args)
        self.last_rangefinder_distance = 0
        self.avoidance = 0
        self.sonar = maxSonar()

    def send_SONAR_message(self, sonar):
        self.msg = self.message_factory.distance_sensor_encode(
            0,      # time system boot, not used
            sonar.min_distance,      # min disance cm
            sonar.max_distance,    # max dist cm
            sonar.distance,   # current dist, int cm
            10,      # type sensor
            0,      # on board id, not used
            0, # orientation: 0...7
            0,      # covariance, not used
            )
        self.send_mavlink(self.msg)
