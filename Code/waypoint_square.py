from __future__ import print_function
import math

class POINT:
	def __init__(self,x,y):
		self.lat = x
		self.lon =y

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
	point1 = get_location_metres(aLocation, -aSize, -aSize)
	point2 = get_location_metres(aLocation, aSize, -aSize)
	point3 = get_location_metres(aLocation, aSize, aSize)
	point4 = get_location_metres(aLocation, -aSize, aSize)	
	temp1 = point1
	f.write(str(temp1.lat) + "," + str(temp1.lon) + '\n')
	for i in range (aSize/seed_distance):
		temp2 = temp1
		for j in range(2*aSize/seed_distance):
			newpoint = get_location_metres(temp2, seed_distance, 0)
			temp2 = newpoint
			f.write(str(temp2.lat) + "," + str(temp2.lon) + '\n')		
		shift1 = get_location_metres(temp2, 0, seed_distance)
		temp2  = shift1
		f.write(str(temp2.lat) + "," + str(temp2.lon) + '\n')
		

		for j in range(2*aSize/seed_distance):
			newpoint = get_location_metres(temp2, -seed_distance, 0)
			temp2 = newpoint
			f.write(str(temp2.lat) + "," + str(temp2.lon) + '\n')
		shift2 = get_location_metres(temp2, 0, seed_distance)
		temp1 = shift2
		f.write(str(temp1.lat) + "," + str(temp1.lon) + '\n')
	for j in range(2*aSize/seed_distance):
			newpoint = get_location_metres(temp1, seed_distance, 0)
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

