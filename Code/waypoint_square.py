from __future__ import print_function
import math

class POINT:
	def __init__(self,x,y):
		self.lat = x
		self.lon = y

def calNewGeoLocationNE(initialPOINT, yNORTH, xEAST):
	"""

	This function is used to calculate new GEO Point which is 'y' meters north and 'x' meters east of a Reference point.
	Knowing the lat/long of reference point and y and x distance , it returns a POINT class object with new location lat/long value.
	This function is an approximation therefore valid over small distances (1m to 1km). It is not valid when calculating points close to the earth poles.

	"""
	#Radius of earth
	earthRad=6378137.0

	#New point offset calculated in radian
	tempLAT = yNORTH/earthRad
	tempLON = xEAST/(earthRad*math.cos(math.pi*initialPOINT.lat/180))

	#Now calculate the new point in decimal degrees
	finalLAT = initialPOINT.lat + (tempLAT * 180/math.pi)
	finalLON = initialPOINT.lon + (tempLON * 180/math.pi)
	finalLOCATION = POINT(finalLAT,finalLON)
	return finalLOCATION

def distanceBetweenTwoGeoPoints(locPOINT1, locPOINT2):
	"""
	This function calulates the ground distance between two points.
	This function is a approximation therefore valid for only short distance.

	"""
	disLatitude = locPOINT2.lat - locPOINT1.lat
	disLongitude = locPOINT2.lon - locPOINT1.lon
	return math.sqrt((disLatitude*disLatitude) + (disLongitude*disLongitude)) * 1.113195e5

def generate_points(aLocation,aSize,seed_distance):
	"""

	This function calculates the waypoints in a spiral manner for plantation of seeds in a square field.
	Input:
		alocation: location of centre of square in lat/lon
		aSize: half the side of square
		seed_distance: distance between two waypoints or seed plantation distance

	Output:
		A txt file named:  waypoint_square.txt, stores the waypoints generated in a sequence.

	"""


	f = open("waypoint_square.txt","w+")
	temp1 = calNewGeoLocationNE(aLocation, -aSize, -aSize)
	f.write(str(temp1.lat) + "," + str(temp1.lon) + '\n')
	for i in range (aSize/seed_distance):
		temp2 = temp1
		for j in range(2*aSize/seed_distance):
			newpoint = calNewGeoLocationNE(temp2, seed_distance, 0)
			temp2 = newpoint
			f.write(str(temp2.lat) + "," + str(temp2.lon) + '\n')
		shift1 = calNewGeoLocationNE(temp2, 0, seed_distance)
		temp2  = shift1
		f.write(str(temp2.lat) + "," + str(temp2.lon) + '\n')


		for j in range(2*aSize/seed_distance):
			newpoint = calNewGeoLocationNE(temp2, -seed_distance, 0)
			temp2 = newpoint
			f.write(str(temp2.lat) + "," + str(temp2.lon) + '\n')
		shift2 = calNewGeoLocationNE(temp2, 0, seed_distance)
		temp1 = shift2
		f.write(str(temp1.lat) + "," + str(temp1.lon) + '\n')
	for j in range(2*aSize/seed_distance):
			newpoint = calNewGeoLocationNE(temp1, seed_distance, 0)
			temp1 = newpoint
			f.write(str(temp1.lat) + "," + str(temp1.lon) + '\n')


print("   This code generates the required waypoint for the drone \n")
print("Please enter the geo location of the centre of your field: \n")
in_lat = 0.0
in_lon = 0.0
while True:
	try:
		in_lat = float(input("Please enter the latitute of centre:\n"))
		if(in_lat<0 or in_lat>90):
			print("Latitude value must be between 0 and 90")
			continue
		in_lon = float(input("Please enter the longitude of centre:\n"))
		if(in_lon<0 or in_lon>180):
			print("Langitude value must be between 0 and 180")
			continue
		break
	except:
		print("Oops!  That was no valid lat/lon.  Try again...")

initial_location = POINT(in_lat,in_lon)
side_size = 0
while True:
	try:
		side_size = int(input("Please enter the distance between the center and field edge (i.e. side/2):\n"))
		break
	except:
		print("Oops!  Please insert positive interger value.  Try again...")

distance = 0
while True:
	try:
		distance = int(input("Please enter the distance between two waypoints:\n"))
		break
	except:
		print("Oops!  That was no valid distance.  Try again...")

generate_points(initial_location,side_size,distance)
print("\n   Waypoints are generated and stored in waypoint_square.txt file. \n")
