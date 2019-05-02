from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Rangefinder
import time
import math
from serial import Serial
from pymavlink import mavutil
from datetime import datetime
from droneVehicle import droneVehicle, maxSonar
#import pymavlink

#cmd = []

teensy_port = '/dev/ttyUSB1'
xbee_port = '/dev/ttyUSB2'

connection_string = '127.0.0.1:14550'
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, vehicle_class=droneVehicle)


def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip())

def custom_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
vehicle.connect_teensy(teensy_port, 9600)
while True:
    while not vehicle.armed:
        time.sleep(2)
    custom_takeoff(10)
    vehicle.complete_mission()













"""
while vehicle.is_armed is False:
    time.sleep(1)


vehicle.wait_for_mode(VehicleMode("AUTO"), 600)

if vehicle.mode.name == VehicleMode("AUTO").name:
    vehicle.home_location = vehicle.location.global_frame
    vehicle.simple_takeoff(15)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=15*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
    while vehicle.mode.name == VehicleMode("AUTO").name:
        nextwaypoint=vehicle.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))

        if nextwaypoint == vehicle.commands.count - 1: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)
    print('Return to launch')
    vehicle.mode = VehicleMode("RTL")
"""
