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
logFile_handler = logging.FileHandler("drone_seed_GUIDED.log")
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
class LAT_LON_ALT:
	def __init__(self,x,y,z):
		self.lon = x
		self.lat = y
        self.alt = z

def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LAT_LON_ALT object containing the latitude/longitude and altitude `dNorth` and `dEast` metres from the specified `original_location`.
	The function is useful when you want to move the vehicle around specifying locations relative to the current vehicle position.
	This function is relatively accurate over small distances (10m within 1km) except close to the poles.
    The function does not change the altitude value
    Reference:
	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters

	"""
	#Radius of "spherical" earth
	earth_radius=6378137.0

	#Coordinate offsets in radians
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

	#New position in decimal degrees
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	new_location = LAT_LON_ALT(newlon,newlat,original_location.alt)
	return new_location

def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LocationGlobal objects.
	This method is an approximation, and will not be accurate over large distances and close to the earth's poles.
	Reference:
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py

	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

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


def goto(targetLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    """
    # send command to vehicle
    vc_in_loc = vehicle.location.global_relative_frame
    vehicle_initialLocation = LAT_LON_ALT(vc_in_loc.lon,vc_in_loc.lat,vc_in_loc.alt)
    targetDistance = get_distance_metres(vehicle_initialLocation, targetLocation)
    msg = vehicle.message_factory.set_position_target_global_int_encode( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000, targetLocation.lat, targetLocation.lon, targetLocation.alt, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        logger.debug("mode: %s" % vehicle.mode.name)
        vc_loc = vehicle.location.global_relative_frame
        vehicle_currentLocation = LAT_LON_ALT(vc_loc.lon,vc_loc.lat,vc_loc.alt)
        remainingDistance=get_distance_metres(vehicle_currentLocation, targetLocation)
        logger.info("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            logger.info("Reached target")
            break;
        time.sleep(2)
