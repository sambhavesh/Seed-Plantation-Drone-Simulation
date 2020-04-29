#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal,Command
import os
import json
import urllib
import math
import argparse
import time
import logging , logging.handlers

#----------------------------------------------------------------------------------------------------------------------------------
#logging configuration:

logging.basicConfig(filename = "Master.log" , level = logging.DEBUG , format = "%(levelname)s: %(filename)s: %(funcName)s: %(lineno)d: 			%(message)s")
logVAR = logging.getLogger(__name__)
logVAR.setLevel(logging.DEBUG)
logFileHand = logging.FileHandler("drone_seed_GUIDED.log")
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
class LAT_LON_ALT:
	def __init__(self,x,y,z):
		self.lon = x
		self.lat = y
		self.alt = z

def distanceBetweenTwoGeoPoints(locPOINT1, locPOINT2):
	"""
	This function calulates the ground distance between two points.
	This function is a approximation therefore valid for only short distance.

	"""
	disLatitude = locPOINT2.lat - locPOINT1.lat
	disLongitude = locPOINT2.lon - locPOINT1.lon
	return math.sqrt((disLatitude*disLatitude) + (disLongitude*disLongitude)) * 1.113195e5

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


def goto(targetLocation):

	# send command to simulated drone
	logVAR.debug("Target location lat: %f , lon: %f , alt: %f" % (targetLocation.lat,targetLocation.lon,targetLocation.alt))
	vc_in_loc = simDRONE.location.global_relative_frame
	simDRONE_initialLocation = LAT_LON_ALT(vc_in_loc.lon,vc_in_loc.lat,vc_in_loc.alt)
	targetDistance = distanceBetweenTwoGeoPoints(simDRONE_initialLocation, targetLocation)
	msg = simDRONE.message_factory.set_position_target_global_int_encode( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000, targetLocation.lat*1e7, targetLocation.lon*1e7, targetLocation.alt, 0, 0, 0, 0, 0, 0, 0, 0)
	simDRONE.send_mavlink(msg)
	logVAR.debug("Send Command Message to drone")
	# target = LocationGlobal(targetLocation.lat,targetLocation.lon,targetLocation.alt)
	# simDRONE.airspeed=15
	# simDRONE.simple_goto(target)

	fiveSecondCheck = targetDistance
	fiveCounter = 1
	logVAR.debug("fiveSecondCheck distance: %f " % (fiveSecondCheck))
	logVAR.debug("fiveCounter value: %d " % (fiveCounter))
	while True:
		logVAR.debug("mode: %s" % simDRONE.mode.name)
		while (simDRONE.mode.name != "GUIDED"):
			simDRONE.mode = VehicleMode("GUIDED")
			time.sleep(0.1)

		if fiveCounter == 1:
			vc_loc = simDRONE.location.global_relative_frame
			simDRONE_currentLocation = LAT_LON_ALT(vc_loc.lon,vc_loc.lat,vc_loc.alt)
			fiveSecondCheck = distanceBetweenTwoGeoPoints(simDRONE_currentLocation, targetLocation)
			logVAR.debug("fiveSecondCheck distance: %f " % (fiveSecondCheck))
			logVAR.debug("fiveCounter value: %d " % (fiveCounter))

		if fiveCounter >=5:
			logVAR.debug("fiveSecondCheck distance: %f " % (fiveSecondCheck))
			logVAR.debug("fiveCounter value: %d " % (fiveCounter))
			fiveCounter = 1
			vc_loc = simDRONE.location.global_relative_frame
			simDRONE_currentLocation = LAT_LON_ALT(vc_loc.lon,vc_loc.lat,vc_loc.alt)
			currentDistanceToTarget = distanceBetweenTwoGeoPoints(simDRONE_currentLocation, targetLocation)
			logVAR.debug("fiveSecondCheck currentDistanceToTarget distance: %f " % (currentDistanceToTarget))
			if currentDistanceToTarget >= 0.9* fiveSecondCheck:
				#resend the msg command to drone
				simDRONE.send_mavlink(msg)
				logVAR.critical("Last command message dropped. Resending the command message to drone")
				logVAR.debug("Resend the command message to drone.")

		vc_loc = simDRONE.location.global_relative_frame
		simDRONE_currentLocation = LAT_LON_ALT(vc_loc.lon,vc_loc.lat,vc_loc.alt)
		remDistance=distanceBetweenTwoGeoPoints(simDRONE_currentLocation, targetLocation)
		logVAR.info("Distance to next seed drop point: %f" % (remDistance))
		print("Distance to next seed drop point: %f" % (remDistance))
		if remDistance <= 1:
			logVAR.info("Reached drop point")
			break
		fiveCounter += 1
		time.sleep(1)





def print_simDRONE_parameters():
	"""
	This function list all the parameters of the simulated drone and stores it in log file
	"""
	logVAR.info ("Listing all the current simulated drone parameters (`simDRONE.parameters`):")
	for para, ivalue in simDRONE.parameters.iteritems():
		logVAR.info (" Parameter : %s Current Value : %s" % (para,ivalue))


def startMission(startingLocation):
	"""
	This function controls the planned mission of drone
	"""

	with open(waypoint_file,"r") as waypointFile:
		for pt in waypointFile:
			current_line = pt.split(",")
			nextLocation = LAT_LON_ALT(float(current_line[1]),float(current_line[0]),startingLocation.alt)
			logVAR.debug("Next location lat: %f , lon: %f , alt: %f",nextLocation.lat,nextLocation.lon,nextLocation.alt)
			goto(nextLocation)
			print("Dropping Seed")
			logVAR.info("Dropping Seed")
	waypointFile.close()
#-------------------------------------------------------------------------------------------------------------------------------------
#Main body:

startingLocation = LAT_LON_ALT(0.0,0.0,0.0) #startingLocation variable
waypoint_file = ""	#stores the waypoint file name

#Takes the lat lon and alt value from USER
while True:
	try:
		startingLocation.lat = float(input("Please enter the latitute of starting point:\n"))
		logVAR.debug("USER entered latitute value: %s",str(startingLocation.lat))
		if(startingLocation.lat<0 or startingLocation.lat>90):
			print("Latitude value must be between 0 and 90")
			continue
		startingLocation.lon = float(input("Please enter the longitude of starting point:\n"))
		logVAR.debug("USER entered longitude value: %s",str(startingLocation.lon))
		if(startingLocation.lon<0 or startingLocation.lon>180):
			print("Langitude value must be between 0 and 180")
			continue
		startingLocation.alt = float(input("Please enter the altitude for the drone:\n"))
		logVAR.debug("USER entered altitude value: %s",str(startingLocation.alt))
		if(startingLocation.alt<0):
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

parsingVAR = argparse.ArgumentParser(description=' Demonstrates Seed Plantation Mission in GUIDED mode.')
parsingVAR.add_argument('--connect', help="simDRONE connection string. SITL is automatically started if connection string not specified.")
argsVAR = parsingVAR.parse_args()
userConString = argsVAR.connect
sitlSIM = None

#Start the SITL is user do not specify the connection string for the drone
if not userConString:
	import dronekit_sitl
	sitlSIM = dronekit_sitl.start_default(lat=startingLocation.lat,lon=startingLocation.lon)
	userConString = sitlSIM.connection_string()

# Now connect to the simulated drone
print('Connecting to simulated drone on: %s' % userConString)
logVAR.info('Connecting to simulated drone on: %s' % userConString)
simDRONE = connect(userConString, wait_ready=True)


#log simulated drone parameters:
print_simDRONE_parameters()

print("Drone is arming and taking off:")
logVAR.info("Drone is arming and taking off:")
armVehicleThenTakeOFF(startingLocation.alt)

print("Starting mission")
logVAR.info("Starting mission")

startMission(startingLocation)


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
