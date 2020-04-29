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
	new_point = calNewGeoLocationNE(first_point,move_north,move_east)
	input_file.write(str(new_point.lat) + "," + str(new_point.lon) + '\n')

print("All generated points are stored in input.txt.\n")
