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
	"""
	Returns a LAT_LON object containing the latitude/longitude `dNorth` and `dEast` metres from the specified `original_location`.
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
	new_location = LAT_LON(newlon,newlat)
	return new_location

def get_distance_metres(aLocation1, aLocation2):
	"""
	Returns the ground distance in metres between two LAT_LON objects.
	This method is an approximation, and will not be accurate over large distances and close to the earth's poles.
	Reference:
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py

	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def generate_points(start_point,edge_size,seed_distance,polygon_hull):

    output_file = open("waypoint_convex.txt","w+")
    func_tempVar1 = start_point
    shapely_tempVar1 = Point(func_tempVar1.lon,func_tempVar1.lat)
    if (polygon_hull.contains(shapely_tempVar1) or shapely_tempVar1.within(polygon_hull) or polygon_hull.touches(shapely_tempVar1) ):
		output_file.write(str(func_tempVar1.lat) + "," + str(func_tempVar1.lon) + '\n')

    step_size = edge_size/seed_distance

    for i in range (step_size/2):
        func_tempVar2 = func_tempVar1
        for j in range(step_size):
            func_newpoint = get_location_metres(func_tempVar2, seed_distance, 0)
            func_tempVar2 = func_newpoint
            shapely_tempVar1 = Point(func_tempVar2.lon,func_tempVar2.lat)
            if (polygon_hull.contains(shapely_tempVar1) or shapely_tempVar1.within(polygon_hull) or polygon_hull.touches(shapely_tempVar1) ):
        		output_file.write(str(func_tempVar2.lat) + "," + str(func_tempVar2.lon) + '\n')
        func_shift1 = get_location_metres(func_tempVar2, 0, seed_distance)
        func_tempVar2 = func_shift1
        shapely_tempVar1 = Point(func_tempVar2.lon,func_tempVar2.lat)
        if (polygon_hull.contains(shapely_tempVar1) or shapely_tempVar1.within(polygon_hull) or polygon_hull.touches(shapely_tempVar1) ):
            output_file.write(str(func_tempVar2.lat) + "," + str(func_tempVar2.lon) + '\n')

        for j in range(step_size):
            func_newpoint = get_location_metres(func_tempVar2, -seed_distance, 0)
            func_tempVar2 = func_newpoint
            shapely_tempVar1 = Point(func_tempVar2.lon,func_tempVar2.lat)
            if (polygon_hull.contains(shapely_tempVar1) or shapely_tempVar1.within(polygon_hull) or polygon_hull.touches(shapely_tempVar1) ):
        		output_file.write(str(func_tempVar2.lat) + "," + str(func_tempVar2.lon) + '\n')

        func_shift2 = get_location_metres(func_tempVar2, 0, seed_distance)
        func_tempVar1 = func_shift2
        shapely_tempVar1 = Point(func_tempVar1.lon,func_tempVar1.lat)
        if (polygon_hull.contains(shapely_tempVar1) or shapely_tempVar1.within(polygon_hull) or polygon_hull.touches(shapely_tempVar1) ):
            output_file.write(str(func_tempVar1.lat) + "," + str(func_tempVar1.lon) + '\n')


    for j in range(step_size):
        func_newpoint = get_location_metres(func_tempVar1, seed_distance, 0)
        func_tempVar1 = func_newpoint
        shapely_tempVar1 = Point(func_tempVar1.lon,func_tempVar1.lat)
        if (polygon_hull.contains(shapely_tempVar1) or shapely_tempVar1.within(polygon_hull) or polygon_hull.touches(shapely_tempVar1) ):
            output_file.write(str(func_tempVar1.lat) + "," + str(func_tempVar1.lon) + '\n')







#------------------------------------------------------------------------------------------------------------------



input_file = ""
while True:
	input_file = raw_input("Enter the file name with extension containing lat long of corners of polygon:\n")
	if os.path.exists(input_file):
		break
	else:
		print("Enter file does not exists. Please re enter correct file")
		continue

seed_distance = 0
while True:
	try:
		seed_distance = int(input("Please enter the distance between two waypoints:\n"))
		break
	except:
		print("Oops!  That was no valid distance.  Try again...")
latlon_list = []
with open(input_file,"r") as input_f:
    for line in input_f:
        current_line = line.split(",")
        latlon_list.append([float(current_line[1]),float(current_line[0])])
latlon_num = np.array(latlon_list )#,dtype = np.float64)
polygon_hull = Polygon(latlon_num)
#print(latlon_list)
# print(latlon_num)
min_col = np.amin(latlon_num,axis = 0)
max_col = np.amax(latlon_num,axis = 0)

#print (min_col)
#print(max_col)

# min Long , min lat
t1 = LAT_LON(min_col[0],min_col[1])
# min Long , max lat
t2 = LAT_LON(min_col[0],max_col[1])
# max Long , min lat
t3 = LAT_LON(max_col[0],min_col[1])

# print(t1.lat,t1.lon)
# print(t2.lat,t2.lon)
# print(t3.lat,t3.lon)



cal_d = max(get_distance_metres(t1,t2),get_distance_metres(t1,t3))
# print(cal_d)
total_step = math.ceil(cal_d/seed_distance)
# print(total_step)
if total_step%2 == 0:
	cal_d = seed_distance*(total_step)
else:
	cal_d = seed_distance*(total_step+1)


start_point = t1
# print(start_point.lat,start_point.lon)
# print(cal_d)
# print(seed_distance)
generate_points(start_point,int(cal_d),seed_distance,polygon_hull)

print("\n   Waypoints are generated and stored in waypoint_square.txt file. \n")
