from __future__ import print_function
import math
import numpy as np
import os
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
np.set_printoptions(precision=12)
class LAT_LON:
	def __init__(self,x,y):
		self.lon = x
		self.lat = y

def get_location_metres(original_location, dNorth, dEast):

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

	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5












input_file = ""
while True:
	input_file = raw_input("Enter the file name with extension containing lat long of corners of polygon:\n")
	if os.path.exists(input_file):
		break
	else:
		print("Enter file does not exists. Please re enter correct file")
		continue
latlon_list = []
with open(input_file,"r") as input_f:
    for line in input_f:
        current_line = line.split(",")
        latlon_list.append([float(current_line[1]),float(current_line[0])])
latlon_num = np.array(latlon_list )#,dtype = np.float64)

#print(latlon_list)
# print(latlon_num)
min_col = np.amin(latlon_num,axis = 0)
max_col = np.amax(latlon_num,axis = 0)


# min Long , min lat
t1 = LAT_LON(min_col[0],min_col[1])
# min Long , max lat
t2 = LAT_LON(min_col[0],max_col[1])
# max Long , min lat
t3 = LAT_LON(max_col[0],min_col[1])
cal_d = max(get_distance_metres(t1,t2),get_distance_metres(t1,t3))

# print(t1.lat,t1.lon)
# print(t2.lat,t2.lon)
# print(t3.lat,t3.lon)







#print (min_col)
#print(max_col)
