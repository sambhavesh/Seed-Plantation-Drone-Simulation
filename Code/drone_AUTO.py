#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import os
import json, urllib, math
import time
import logging , logging.handlers

#----------------------------------------------------------------------------------------------------------------------------------
#logging configuration:

logging.basicConfig(filename = "Master.log" , level = logging.DEBUG , format = "%(levelname)s: %(filename)s: %(module)s: %(funcName)s: %(lineno)d: 			%(message)s")
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logFile_handler = logging.FileHandler("drone_seed.log")
logFile_handler.setLevel(logging.DEBUG)
logFile_streamHandler = logging.StreamHandler()
logFile_streamHandler.setLevel(logging.ERROR)
logging_formatter = logging.Formatter("%(levelname)s: %(filename)s: %(module)s: %(funcName)s: %(lineno)d: 			%(message)s")
logFile_handler.setFormatter(logging_formatter)
logFile_streamHandler.setFormatter(logging_formatter)
logger.addHandler(logFile_handler)
logger.addHandler(logFile_streamHandler)

#-------------------------------------------------------------------------------------------------------------------------------------
#Required function defination:

def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.
	This method is an approximation, and will not be accurate over large distances and close to the
	earth's poles.
	Reference:
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
	Arms vehicle and fly to a target altitude.
	"""

	# Don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		logger.warning(" Waiting for vehicle to initialise...")
		time.sleep(1)
	# Set mode to GUIDED for arming and takeoff:
	while (vehicle.mode.name != "GUIDED"):
		vehicle.mode = VehicleMode("GUIDED")
		time.sleep(0.1)
	# Confirm vehicle armed before attempting to take off
	while not vehicle.armed:
		vehicle.armed = True
		logger.warning(" Waiting for arming...")
		time.sleep(1)
	print(" Taking off!")
	logger.info("Taking off!")
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Wait until the vehicle reaches a safe height
	# before allowing next command to process.
	while True:
		requiredAlt = aTargetAltitude*0.95
		#Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt>=requiredAlt:
			print(" Reached target altitude of ~%f" % (aTargetAltitude))
			logger.info(" Reached target altitude of ~%f" % (aTargetAltitude))
			break
		logger.info(" Altitude: %f < %f" % (vehicle.location.global_relative_frame.alt,requiredAlt))
		time.sleep(1)

def print_vechicle_attributes():
	"""
	This function list all the attributes of the vechicle and stores it in log file
	"""
	logger.info("Autopilot Firmware version: %s" % vehicle.version)
	logger.info("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
	logger.info("Global Location:INFO:__main__: Key:BATT_CURR_PIN Value:12.0 %s" % vehicle.location.global_frame)
	logger.info("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
	logger.info("Local Location: %s" % vehicle.location.local_frame)
	logger.info("Attitude: %s" % vehicle.attitude)
	logger.info("Velocity: %s" % vehicle.velocity)
	logger.info("GPS: %s" % vehicle.gps_0)
	logger.info("Groundspeed: %s" % vehicle.groundspeed)
	logger.info("Airspeed: %sINFO:__main__:Distance to waypoint (2): 50.5458561177" % vehicle.airspeed)
	logger.info("Gimbal status: %s" % vehicle.gimbal)
	logger.info("Battery: %s" % vehicle.battery)
	logger.info("EKF OK?: %s" % vehicle.ekf_ok)
	logger.info("Last Heartbeat: %s" % vehicle.last_heartbeat)
	logger.info("Rangefinder: %s" % vehicle.rangefinder)
	logger.info("Rangefinder distance: %s" % vehicle.rangefinder.distance)
	logger.info("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
	logger.info("Heading: %s" % vehicle.heading)
	logger.info("Is Armable?: %s" % vehicle.is_armable)
	logger.info("System status: %s" % vehicle.system_status.state)
	logger.info("Mode: %s" % vehicle.mode.name)
	logger.info("Armed: %s" % vehicle.armed)

def print_vechicle_parameters():
	"""
	This function list all the parameters of the vechicle and stores it in log file
	"""
	logger.info ("Print all parameters (`vehicle.parameters`):")
	for key, value in vehicle.parameters.iteritems():
		logger.info (" Key:%s Value:%s" % (key,value))

#-------------------------------------------------------------------------------------------------------------------------------------
#Main body:

start_lat = 0.0		#latitute variable
start_lon = 0.0		#longitude variable
start_alt = 0.0		#altitude variable
waypoint_file = ""	#stores the waypoint file name

#Takes the lat lon and alt value from USER
while True:
	try:
		start_lat = float(input("Please enter the latitute of starting point:\n"))
		logger.debug("USER entered latitute value: %s",str(start_lat))
		if(start_lat<0 or start_lat>90):
			print("Latitude value must be between 0 and 90")
			continue
		start_lon = float(input("Please enter the longitude of starting point:\n"))
		logger.debug("USER entered longitude value: %s",str(start_lon))
		if(start_lon<0 or start_lon>180):
			print("Langitude value must be between 0 and 180")
			continue
		start_alt = float(input("Please enter the altitude for the drone:\n"))
		logger.debug("USER entered altitude value: %s",str(start_alt))
		if(start_alt<0):
			print("Altitude value must be positive")
			continue
		break
	except:
		logger.error("Oops!  That was no valid lat/lon or altitude.  Try again...")

#Takes the waypoint file name from USER
while True:
	waypoint_file = raw_input("Enter the waypoint file name with extension:\n")
	if os.path.exists(waypoint_file):
		break
	else:
		print("Enter file does not exists. Please re enter correct file")
		logger.error("Enter file does not exists.")
		continue


#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates Seed Plantation Mission.')
parser.add_argument('--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
	import dronekit_sitl
	sitl = dronekit_sitl.start_default(lat=start_lat,lon=start_lon)
	connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
logger.info('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#log vechicle attributes:
print_vechicle_attributes()
#log vechicle parameters:
print_vechicle_parameters()

# Now download the vehicle waypoints
cmds = vehicle.commands
cmds.wait_ready()
cmds = vehicle.commands
cmds.clear()
line_count = 0    #Variable that keep track of total commands

#Add command for starting location:
cmd = Command( 0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 0, 0, 0, 0,start_lat, start_lon,start_alt)
cmds.add(cmd)

#Add command for all waypoints:
with open(waypoint_file,"r") as way_p:
	for pt in way_p:
		current_line = pt.split(",")
		line_count +=1
		lat = float(current_line[0])
		lon = float(current_line[1])
		logger.debug ("Point: %f %f" %(lat, lon))
		cmd = Command( 0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 5, 0, 0, 0,lat, lon, start_alt)
		cmds.add(cmd)
		""""
		Add the codes/ mechanism for dropping seed here. Depends on hardware

		"""
way_p.close()

#Add command for returing to base:
cmd = Command( 0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 0, 0, 0, 0,start_lat, start_lon,start_alt)
cmds.add(cmd)

#Upload clear message and command messages to vehicle.
print("Uploading waypoints to vehicle..." )
logger.info("Uploading waypoints to vehicle...")
cmds.upload()
print("Arm and Takeoff")
logger.info("Arm and Takeoff")
arm_and_takeoff(start_alt)

print("Starting mission")
logger.info("Starting mission")

# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission:
while (vehicle.mode.name != "AUTO"):
    vehicle.mode = VehicleMode("AUTO")
    time.sleep(0.1)


# Monitor mission then RTL (Return to launch) and quit:
while True:
	nextwaypoint=vehicle.commands.next
	print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
	logger.info('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
	if distance_to_current_waypoint()<1.5:
		print("Dropping Seed")
		logger.critical("Dropping Seed")
	if nextwaypoint==line_count+1:
		print("Exit 'standard' mission when start heading to final waypoint or start location")
		logger.info("Exit 'standard' mission when start heading to final waypoint or start location")
		break;
	time.sleep(1)

print('Return to launch')
logger.critical("Return to launch")
while (vehicle.mode.name != "RTL"):
	vehicle.mode = VehicleMode("RTL")
	time.sleep(0.1)

#Close vehicle object before exiting script
print("Close vehicle object")
logger.info("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
	sitl.stop()
print("Completed...")
logger.info("Completed...")

'''
sample input:

10.0
10.0
30.0
waypoint_square.txt
'''
