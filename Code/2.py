#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function

from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import json, urllib, math
import time

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



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

    
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    # Set mode to GUIDED for arming and takeoff:
    while (vehicle.mode.name != "GUIDED"):
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(0.1)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        vehicle.armed = True
        print(" Waiting for arming...")
        time.sleep(1)

    print(" Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height 
    # before allowing next command to process.
    while True:
        requiredAlt = aTargetAltitude*0.95
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=requiredAlt: 
            print(" Reached target altitude of ~%f" % (aTargetAltitude))
            break
        print(" Altitude: %f < %f" % (vehicle.location.global_relative_frame.alt,
                                      requiredAlt))
        time.sleep(1)


start_lat = 0.0
start_lon = 0.0
start_alt = 0.0
waypoint_file = ""
while True:
	try:
		start_lat = float(input("Please enter the latitute of centre:\n"))
		if(start_lat<0 or start_lat>90):
			print("Latitude value must be between 0 and 90")			
			continue
		start_lon = float(input("Please enter the longitude of centre:\n"))
		if(start_lon<0 or start_lon>180):
			print("Langitude value must be between 0 and 180")
			continue
		start_alt = float(input("Please enter the altitude for the drone:\n"))
		if(start_alt<0):
			print("Altitude value must be positive")
			continue				
		break
	except:
		print("Oops!  That was no valid lat/lon or altitude.  Try again...")
waypoint_file = raw_input("Enter the waypoint file name with extension:\n")






#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default(lat=start_lat,lon=start_lon)
    connection_string = sitl.connection_string()



#------------------------------------------------------------------------------------------------------------






# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
# Now download the vehicle waypoints
cmds = vehicle.commands
cmds.wait_ready()
cmds = vehicle.commands
cmds.clear()
line_count = 0
cmd = Command( 0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 0, 0, 0, 0,start_lat, start_lon,start_alt)
cmds.add(cmd)
with open(waypoint_file,"r") as way_p:
	for pt in way_p:
		current_line = pt.split(",")
		line_count +=1		
		lat = float(current_line[0])
		lon = float(current_line[1])
		print ("Point: %f %f" %(lat, lon))
		cmd = Command( 0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 5, 0, 0, 0,lat, lon, start_alt)
		cmds.add(cmd)
way_p.close()
cmd = Command( 0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 0, 0, 0, 0,start_lat, start_lon,start_alt)
cmds.add(cmd)
#Upload clear message and command messages to vehicle.
print("Uploading waypoints to vehicle..." ) 20.3429490123
Distance to waypoint (2): 15.7119687615
CRITICAL:autopilot:Reached Command #2
Distance to waypoint (3): 59.
cmds.upload()
print("Arm and Takeoff")
arm_and_takeoff(start_alt)


print("Starting mission")

# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission:
while (vehicle.mode.name != "AUTO"):
    vehicle.mode = VehicleMode("AUTO")
    time.sleep(0.1)

# Monitor mission for 60 seconds then RTL and quit:

while True:
	nextwaypoint=vehicle.commands.next
	print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
	if distance_to_current_waypoint()<1.5:
		print("Dropping Seed")
	if nextwaypoint==line_count+1:
		print("Exit 'standard' mission when start heading to final waypoint or start location")
		break;
	time.sleep(1)

print('Return to launch')
while (vehicle.mode.name != "RTL"):
	vehicle.mode = VehicleMode("RTL")
	time.sleep(0.1)

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed...")





'''
sample input:

10.0
10.0
30.0
waypoint_square.txt
'''
