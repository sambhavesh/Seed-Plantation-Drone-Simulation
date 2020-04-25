from __future__ import print_function
import math


class POINT:
	def __init__(self,x,y):
		self.lat = x
		self.lon = y

def get_location_metres(original_location, dNorth, dEast):
	"""
	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the specified `original_location`.
	The function is useful when you want to move the vehicle around specifying locations relative to the current vehicle position.
	This function is relatively accurate over small distances (10m within 1km) except close to the poles.
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
	new_location = POINT(newlat,newlon)
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



print("   This code generates the input lat long values \n")
print("Please enter the geo location of the reference point of your field: \n")
in_lat = 0.0
in_lon = 0.0
while True:
	try:
		in_lat = float(input("Please enter the latitute of point:\n"))
		if(in_lat<0 or in_lat>90):
			print("Latitude value must be between 0 and 90")
			continue
		in_lon = float(input("Please enter the longitude of point:\n"))
		if(in_lon<0 or in_lon>180):
			print("Langitude value must be between 0 and 180")
			continue
		break
	except:
		print("Oops!  That was no valid lat/lon.  Try again...")

first_point = POINT(in_lat,in_lon)
input_file = open("input.txt","w+")
input_file.write(str(first_point.lat) + "," + str(first_point.lon) + '\n')
number_of_points = int(input("Please enter the more number of point to generate:\n"))
for i in range(number_of_points):
    print("Enter the next point distance in meter (north,east) from refrence point:\n")
    move_north = int(input("North distance to point: "))
    move_east = int(input("East distance to point: "))
    new_point = get_location_metres(first_point,move_north,move_east)
    input_file.write(str(new_point.lat) + "," + str(new_point.lon) + '\n')

print("All generated points are stored in input.txt.\n")
