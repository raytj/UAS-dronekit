from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Rangefinder
import time
import math
from serial import Serial
from pymavlink import mavutil
from datetime import datetime
from droneVehicle import droneVehicle, maxSonar
#import pymavlink

#cmd = []

PORT = 'COM13'


#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default(33.1945624862421, -87.4813413619995)
    connection_string = sitl.connection_string()
    connection_string = '127.0.0.1:14550'

proxy_string = '127.0.0.1:14550'
gcs_string = '192.168.17.1:14551'
# Connect to the Vehicle
print('Connecting to vehicle on: %s through %s' % (connection_string, proxy_string))
#cmd.append("MAVProxy")
#cmd.extend(["--master", connection_string])
#cmd.extend(["--out", proxy_string])
#cmd.extend(["--out", gcs_string])
vehicle = connect(proxy_string, wait_ready=True, vehicle_class=droneVehicle)
#vehicle.parameters['RNGFND1_TYPE'] = 10
#vehicle.parameters['RNGFND1_MAX_CM'] = 640
#vehicle.parameters['RNGFND1_MIN_CM'] = 24
#vehicle.parameters['RNGFND1_SCALE'] = 1.98

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def simulateObject(lat, lon):
    alocation1 = LocationGlobalRelative(lat, lon, 30)
    distance = get_distance_metres(alocation1, vehicle.location.global_frame) * 100
    if distance > 640:
        distance = 640
    return distance
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)


def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip())

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

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

teensy = Serial()
teensy.port = PORT
teensy.baud = 9600
teensy.open()
teensy.flushInput()

import_mission_filename = 'mpmission.txt'
export_mission_filename = 'exportedmission.txt'
vehicle.wait_for_armable(600)
while not vehicle.armed:
    time.sleep(2)
arm_and_takeoff(10)
vehicle.mode = VehicleMode('AUTO')
def range_sensor(dist):

    msg = vehicle.message_factory.distance_sensor_encode(
        0,      # time system boot, not used
        24,      # min disance cm
        640,    # max dist cm
        dist,   # current dist, int cm
        0,      # type sensor
        0,      # on board id, not used
        0, # orientation: 0...7
        0,      # covariance, not used
        )

    vehicle.send_mavlink(msg)
last_rangefinder_measurement = 630
waypoint = vehicle.commands.next
cmds = vehicle.commands
valid_modes = (VehicleMode('AUTO'), VehicleMode('GUIDED'))

s1 = 640
s2 = 640
temperature = 0
pressure = 0
numAvoids = 0
while True:

    #print('rangefinder debug: %s' % rangefinder_measurement)
    if teensy.in_waiting > 12:
        st = teensy.readline().decode('ascii')
        values = st.split(',')
        s1 = int(values[0])
        s2 = int(values[1])
        temperature = int(values[2])
        pressure = float(values[3])
    if  s1 < 400:
        vehicle.mode = VehicleMode('BRAKE')
        print('CRITICAL - Object Collision Imminent! Vehicle Mode Changed to BRAKE')
        time.sleep(1)
        vehicle.wait_ready()
        vehicle.commands.clear()
        vehicle.wait_ready()
        avoid_object = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt + 5)
        vehicle.mode = VehicleMode('GUIDED')
        vehicle.simple_goto(avoid_object)
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt>=avoid_object.alt*0.95 or vehicle.location.global_relative_frame.alt > 40: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(2)
        save_mission('Current_mission.txt')
        #Clear existing mission from vehicle
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        missionlist = readmission('Current_mission.txt')
        for command in missionlist:
            cmds.add(command)
        vehicle.commands.upload()
        vehicle.commands.next = waypoint + 1
        vehicle.simple_goto(LocationGlobalRelative(cmds[waypoint].x, cmds[waypoint].y, cmds[waypoint].z))
        vehicle.mode = VehicleMode('AUTO')
        print('Obstical Avoided! Changing mode back to AUTO')
        vehicle.commands.next = waypoint + 1
    if waypoint < vehicle.commands.next:
        waypoint = vehicle.commands.next
        #range_sensor(last_rangefinder_measurement)
        try:
            missionitem=vehicle.commands[waypoint - 1] #commands are zero indexed
            lat = missionitem.x
            lon = missionitem.y
            alt = missionitem.z
            targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
            print('Heading for location: %s, %s, %s' % (lat, lon, alt))
        except IndexError:
            vehicle.mode = VehicleMode('RTL')
            continue
    try:
        missionitem=vehicle.commands[waypoint - 1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    except IndexError:
        vehicle.mode = VehicleMode('RTL')
        continue
    print('Distance to waypoint (%s): %s' % (waypoint, get_distance_metres(vehicle.location.global_frame, targetWaypointLocation) * 10))
    print('current rangefinder distance: %d' % s1)
    time.sleep(0.1)
    if not vehicle.mode in valid_modes:
        vehicle.mode = VehicleMode('RTL')










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
