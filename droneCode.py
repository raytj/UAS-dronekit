from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
from datetime import datetime
from droneVehicle import droneVehicle, maxSonar

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
###
#if not connection_string:
    #import dronekit_sitl
    #sitl = dronekit_sitl.start_default(33.1945624862421, -87.4813413619995)
    #connection_string = sitl.connection_string()
	#connection_string = '/dev/bin/ttyAMA0'
###
proxy_string = 'udp:192.168.17.1:14550'
# Connect to the Vehicle
print('Connecting to vehicle on: %s through %s' % (connection_string, proxy_string))
vehicle = connect(proxy_string, wait_ready=True, vehicle_class=droneVehicle)
#vehicle.parameters['RNGFND1_TYPE'] = 10
#vehicle.parameters['RNGFND1_MAX_CM'] = 640
#vehicle.parameters['RNGFND1_MIN_CM'] = 24
#vehicle.parameters['RNGFND1_SCALE'] = 1.98
last_rangefinder_distance = 0

@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self, name, message):
    if self.last_rangefinder_distance == round(self.rangefinder.distance, 1):
        return
    self.last_rangefinder_distance = round(self.rangefinder.distance, 1)
    if self.rangefinder.distance <= 300 and self.mode == VehicleMode('AUTO') or self.mode == VehicleMode('GUIDED'):
        self.mode = VehicleMode('BRAKE')
        print("Collision imminent, stopping drone...")
    print("Rangefinder (metres): %s" % last_rangefinder_distance)
vehicle.add_attribute_listener('rangefinder', rangefinder_callback)
@vehicle.on_attribute('mode')
def avoidance_callback(self, attr_name, value):
    if self.mode == VehicleMode('BRAKE'):
        self.mode = VehicleMode('RTL')



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


import_mission_filename = 'mpmission.txt'
export_mission_filename = 'exportedmission.txt'

sonar1 = maxSonar(24, 640)
vehicle.sonar = sonar1

upload_mission(import_mission_filename)
vehicle.mode = VehicleMode('GUIDED')
arm_and_takeoff(40)
vehicle.mode = VehicleMode('AUTO')
vehicle.sonar.update(630)
waypoint = vehicle.commands.next
while True:
    if waypoint < vehicle.commands.next:
        vehicle.sonar.update(vehicle.sonar.distance - 25)
        vehicle.send_SONAR_message(vehicle.sonar)




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
