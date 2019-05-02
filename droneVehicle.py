from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
import time
import math
from serial import Serial
from digi.xbee import devices,
from pymavlink import mavutil
from datetime import datetime



class Sonar(object):
    def __init__(self, id, max_distance=640, min_distance=20, orientation=0, trigger_distance=400):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.orientation = orientation
        self.id = id
        self.time_boost_ms = 0
        self. type = 10
        self.distance = 0
        self.trigger_distance = trigger_distance
        self.triggers = 0
    def update(self, distance):
        if self.distance - trigger_distance < 50:
            self.triggers += 1
        elif self.triggers > 0:
            self.triggers -=1
        self.distance = distance

    def is_triggered(self):
        if self.triggers > 4 and self.distance < self.trigger_distance:
            return True
        if (self.distance + 50) < self.trigger_distance:
            return True
        return False


class DroneVehicle(Vehicle):
    def __init__(self, *args):
        super(droneVehicle, self).__init__(*args)
        self.s1 = Sonar(1, 640, 20, 0, 400)
        self.s2 = Sonar(2, 640, 20, 24, 200)
        self.teensy = Serial()
        self.missionlist = []
        self.next_waypoint = 0

    def add_sonar(self, sonar):
        self.sonarList.append(sonar)

    def connect_teensy(self, port, baud):
        teensy.port = port
        teensy.baud = baud
        teensy.open()
        teensy.flush_input()

    def get_teensy_message(self):
        st = teensy.readline().decode('ascii')
        values = st.split(',')
        self.s1.set_range(int(values[0]))
        self.s2.set_range(int(values[1]))
        self.temperature = int(values[2])
        self.pressure = float(values[3])
        if self.s1.is_triggered():
            self.avoid_collision

    def readmission(self, aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = self.commands
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

    def download_mission(self):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = self.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

    def save_mission(self, aFileName):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)
    #Download mission from vehicle
    missionlist = self.download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = self.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)

    def avoid_collision(height):
        self.mode = VehicleMode('BRAKE')
        print('CRITICAL - Object Collision Imminent! Vehicle Mode Changed to BRAKE')
        self.wait_for_mode('BRAKE')
        self.wait_ready()
        self.save_mission('~/pi/Documents/DroneCode/current_mission.txt')
        self.commands.clear()
        self.wait_ready()
        if self.collsions_avoided > 5:
            self.mode = ('RTL')
            self.wait_for_mode('RTL')
            self.collisions_avoided = 0
            return
        self.collisions_avoided += 1
        adjusted_alt = self.location.global_relative_frame.alt + height
        avoid_object = LocationGlobalRelative(self.location.global_relative_frame.lat, self.location.global_relative_frame.lon, adjusted_alt)
        self.mode = VehicleMode('GUIDED')
        self.simple_goto(avoid_object)
        while True:
            print(" Altitude: ", self.location.global_relative_frame.alt)
            if self.location.global_relative_frame.alt>=avoid_object.alt*0.95 or self.location.global_relative_frame.alt > adjusted_alt: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(2)

    def complete_mission(self)
        while self.next_waypoint < len(self.mission_list):
            self.get_teensy_message()
            self.simple_goto(mission_list[self.next_waypoint])
            while True:
                self.get_teensy_message()
                if get_distance_metres(LocationGlobalRelative(self.location.global_relative_frame.lat, self.location.global_relative_frame.lon, self.location.global_relative_frame.alt), \
                LocationGlobalRelative(self.mission_list[self.next_waypoint].x, self.mission_list[self.next_waypoint].y, self.mission_list[self.next_waypoint].z)) < 1:
                    break
            self.next_waypoint += 1
        self.mode = VehicleMode('RTL')

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
