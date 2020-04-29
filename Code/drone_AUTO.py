#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import os
import json
import urllib
import math
import time
import argparse
import logging , logging.handlers

#----------------------------------------------------------------------------------------------------------------------------------
#logging configuration:

logging.basicConfig(filename = "Master.log" , level = logging.DEBUG , format = "%(levelname)s: %(filename)s: %(funcName)s: %(lineno)d: 			%(message)s")
logVAR = logging.getLogger(__name__)
logVAR.setLevel(logging.DEBUG)
logFileHand = logging.FileHandler("drone_seed_AUTO.log")
logFileHand.setLevel(logging.DEBUG)
logFile_streamHandler = logging.StreamHandler()
logFile_streamHandler.setLevel(logging.ERROR)
logForVAR = logging.Formatter("%(levelname)s: %(filename)s: %(funcName)s: %(lineno)d: 			%(message)s")
logFileHand.setFormatter(logForVAR)
logFile_streamHandler.setFormatter(logForVAR)
logVAR.addHandler(logFileHand)
logVAR.addHandler(logFile_streamHandler)

#-------------------------------------------------------------------------------------------------------------------------------------
#Required function defination:

def distanceBetweenTwoGeoPoints(locPOINT1, locPOINT2):
	"""
	This function calulates the ground distance between two points.
	This function is a approximation therefore valid for only short distance.

	"""
	disLatitude = locPOINT2.lat - locPOINT1.lat
	disLongitude = locPOINT2.lon - locPOINT1.lon
	return math.sqrt((disLatitude*disLatitude) + (disLongitude*disLongitude)) * 1.113195e5

def calculateDistanceToCurrentPoint():
	"""
	This function returns the distance to the current waypoint in meters.
	If Home Location is given as input the function returns None

	"""
	nextVehCommandInt = simDRONE.commands.next
	if nextVehCommandInt==0:
		return None
	nextVehCommand=simDRONE.commands[nextVehCommandInt-1] #zero index
	latitute = nextVehCommand.x
	longitude = nextVehCommand.y
	altitude = nextVehCommand.z
	nextPointLoc = LocationGlobalRelative(latitute,longitude,altitude)
	disPoint = distanceBetweenTwoGeoPoints(simDRONE.location.global_frame, nextPointLoc)
	return disPoint


def armVehicleThenTakeOFF(flyingALT):
	"""
	This function takes a altitude as a parameter then arms the simulated drone and then fly to the given altitude.

	"""

	# Wait for autopilot to get ready
	while not simDRONE.is_armable:
		logVAR.warning("Waiting for drone to get ready -----")
		time.sleep(1.0)

	# Change the mode of drone to GUIDED :
	while (simDRONE.mode.name != "GUIDED"):
		simDRONE.mode = VehicleMode("GUIDED")
		time.sleep(0.2)
	# Now we confirm that simulated drone is armed before taking off
	while not simDRONE.armed:
		simDRONE.armed = True
		logVAR.warning("Wait for simulated drone to get armed")
		time.sleep(0.5)

	print("Simulate Drone is taking off..")
	logVAR.info("Simulate Drone is taking off..")
	simDRONE.simple_takeoff(flyingALT)

	# Now we add a check to see whether drone has reached the safe height:

	while True:
		height = flyingALT*0.95
		if simDRONE.location.global_relative_frame.alt >= height:
			print("Drone has reached the height of %f" % (flyingALT))
			logVAR.info("Drone has reached the height of %f" % (flyingALT))
			break
		logVAR.info("Height: %f < %f" % (simDRONE.location.global_relative_frame.alt,height))
		time.sleep(1.0)



def print_simDRONE_parameters():
	"""
	This function list all the parameters of the simulated drone and stores it in log file
	"""
	logVAR.info ("Listing all the current simulated drone parameters (`simDRONE.parameters`):")
	for para, ivalue in simDRONE.parameters.iteritems():
		logVAR.info (" Parameter : %s Current Value : %s" % (para,ivalue))

#-------------------------------------------------------------------------------------------------------------------------------------
#Main body:

startingLatitude = 0.0		#latitute variable
startingLongitude = 0.0		#longitude variable
startingAltitude = 0.0		#altitude variable
waypoint_file = ""	#stores the waypoint file name

#Takes the lat lon and alt value from USER
while True:
	try:
		startingLatitude = float(input("Please enter the latitute of starting point:\n"))
		logVAR.debug("USER entered latitute value: %s",str(startingLatitude))
		if(startingLatitude<0 or startingLatitude>90):
			print("Latitude value must be between 0 and 90")
			continue
		startingLongitude = float(input("Please enter the longitude of starting point:\n"))
		logVAR.debug("USER entered longitude value: %s",str(startingLongitude))
		if(startingLongitude<0 or startingLongitude>180):
			print("Langitude value must be between 0 and 180")
			continue
		startingAltitude = float(input("Please enter the altitude for the drone:\n"))
		logVAR.debug("USER entered altitude value: %s",str(startingAltitude))
		if(startingAltitude<0):
			print("Altitude value must be positive")
			continue
		break
	except:
		logVAR.error("Oops!  That was no valid lat/lon or altitude.  Try again...")

#Takes the waypoint file name from USER
while True:
	waypoint_file = raw_input("Enter the waypoint file name with extension:\n")
	if os.path.exists(waypoint_file):
		break
	else:
		print("Enter file does not exists. Please re enter correct file")
		logVAR.error("Enter file does not exists.")
		continue


#Now we set up parsing to  get the connection string from user

parsingVAR = argparse.ArgumentParser(description=' Seed Plantation using drone.')
parsingVAR.add_argument('--connect', help="simDRONE connection string. SITL is automatically started if connection string not specified.")
argsVAR = parsingVAR.parse_args()
userConString = argsVAR.connect
sitlSIM = None


#Start the SITL is user do not specify the connection string for the drone
if not userConString:
	import dronekit_sitl
	sitlSIM = dronekit_sitl.start_default(lat=startingLatitude,lon=startingLongitude)
	userConString = sitlSIM.connection_string()

# Now connect to the simulated drone
print('Connecting to simulated drone on: %s' % userConString)
logVAR.info('Connecting to simulated drone on: %s' % userConString)
simDRONE = connect(userConString, wait_ready=True)


#log drone parameters:
print_simDRONE_parameters()

# Download the simulated drone commands
droneCMDS = simDRONE.commands
droneCMDS.wait_ready()
droneCMDS = simDRONE.commands
droneCMDS.clear()
line_count = 0    #Variable that keep track of total commands

#Add command for starting location:
droneCMD = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT , mavutil.mavlink.MAV_CMD_NAV_WAYPOINT , 0, 0, 0, 0, 0, 0,startingLatitude, startingLongitude,startingAltitude)
droneCMDS.add(droneCMD)

#Add command for all waypoints:
with open(waypoint_file,"r") as way_p:
	for pt in way_p:
		current_line = pt.split(",")
		line_count +=1
		lat = float(current_line[0])
		lon = float(current_line[1])
		logVAR.debug ("Point: %f %f" %(lat, lon))
		droneCMD = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT , mavutil.mavlink.MAV_CMD_NAV_WAYPOINT , 0, 0, 5, 0, 0, 0,lat, lon, startingAltitude)
		droneCMDS.add(droneCMD)
		""""
		Add the codes/ mechanism for dropping seed here. Depends on hardware

		"""
way_p.close()

#Add command for returing to base:
droneCMD = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0, 0, 0, 0, 0, 0,startingLatitude, startingLongitude,startingAltitude)
droneCMDS.add(droneCMD)

# Now we upload all the waypoints to our simulated drone.
print("Upload points to simulated drone..." )
logVAR.info("Upload points to simulated drone...")
droneCMDS.upload()
print("Drone is arming and taking off:")
logVAR.info("Drone is arming and taking off:")
armVehicleThenTakeOFF(startingAltitude)

print("Starting Seed Plantation mission")
logVAR.info("Starting Seed Plantation mission")

# Reset command to first point i.e 0
simDRONE.commands.next=0

# To start the mission set the drone MODE to AUTO:
while (simDRONE.mode.name != "AUTO"):
	simDRONE.mode = VehicleMode("AUTO")
	time.sleep(0.1)


# Monitor mission then RTL (Return to launch) and quit:
while True:
	nextVehCommandInt = simDRONE.commands.next
	print('Distance to next seed drop point (%s): %s' % (nextVehCommandInt, calculateDistanceToCurrentPoint()))
	logVAR.info('Distance to next seed drop point (%s): %s' % (nextVehCommandInt, calculateDistanceToCurrentPoint()))
	if calculateDistanceToCurrentPoint()<1.5:
		print("Dropping Seed")
		logVAR.critical("Dropping Seed")
	if nextVehCommandInt==line_count+1:
		print("Drone is heading to start location or launch location")
		logVAR.info("Drone is heading to start location or launch location")
		break;
	time.sleep(1)

print('Return to base/helipad')
logVAR.critical("Return to base/helipad")
while (simDRONE.mode.name != "RTL"):
	simDRONE.mode = VehicleMode("RTL")
	time.sleep(0.1)

#Close simulated drone object before terminating script
print("Close simulated drone object")
logVAR.info("Close simulated drone object")
simDRONE.close()

# Shut down simulator
if sitlSIM is not None:
	sitlSIM.stop()
print("Seed Plantation Completed...")
logVAR.info("Seed Plantation Completed...")

'''
sample input:

10.0
10.0
30.0
waypoint_square.txt
'''
