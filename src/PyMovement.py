import time
import math
import argparse
# from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

#Connect to the Vehicle
#connectionFunc()
"""
connection_string = '/dev/ttyUSB0, 57600'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
print("Connected")
"""

vehicle = None

def connectionFunc():
	# global vehicle
	# connection_string = '/dev/ttyUSB0, 57600'
	# print("Connecting to vehicle on: %s" % (connection_string,))
	# vehicle = connect(connection_string, wait_ready=True)
	print("Connected")

# def move(rightDist, upDist):
# 	print("Right Distance: %2.5f", rightDist)	#Print the Y Distance
# 	print("Up Distance: %2.5f", upDist)	#Print the Z Distance\
# 	goto(0, rightDist, upDist)

# def rotate():
# 	print("Rotating in place. No section selected.")	#Print an indication that no section was selected
# 	goto(-1, 0, 0)

# def stop():
# 	print("UAV is stopped because the center section is clear.")	#Print an indication that the center section was selected
# 	goto(1, 0, 0)

def arm():
	global vehicle
	print ("Basic pre-arm checks")
	# # Don't try to arm until autopilot is ready
	# while not vehicle.is_armable:
	# 	print " Waiting for vehicle to initialise..."
	# 	time.sleep(1)

	# print "Arming motors"
	# # Copter should arm in GUIDED mode
	# vehicle.mode = VehicleMode("GUIDED")
	# vehicle.armed = True
	# time.sleep(5)

	# # Confirm vehicle armed before attempting to take off
	# while not vehicle.armed:
	# 	print " Waiting for arming..."
	# 	time.sleep(10)

#### Altitude in meters (Unsure of referrence altitude)
def takeoff(aTargetAltitude):
	# global vehicle
	print ("Taking off at")
	print (aTargetAltitude)
	# print ("Taking off at " + aTargetAltitude)
	# vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# #Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
	# # after Vehicle.simple_takeoff will execute immediately).
	# while True:
	# 	#print " Altitude: ", vehicle.location.global_relative_frame.alt
	# 	#Break and return from function just below target altitude.
	# 	if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
	# 		print "Reached target altitude"
	# 		break
	# time.sleep(1)
	# print "Set default/target airspeed to 1"
	# vehicle.airspeed = 3

# def get_location_metres(original_location, dNorth, dEast, alt):
# 	global vehicle
# 	"""
# 	Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
# 	specified `original_location`. The altitude is adjusted based on 'alt' meteres

# 	The function is useful when you want to move the vehicle around specifying locations relative to 
# 	the current vehicle position.

# 	The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

# 	For more information see:
# 	http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
# 	"""
# 	earth_radius = 6378137.0 #Radius of "spherical" earth
# 	#Coordinate offsets in radians
# 	dLat = dNorth/earth_radius
# 	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

# 	#New position in decimal degrees
# 	newlat = original_location.lat + (dLat * 180/math.pi)
# 	newlon = original_location.lon + (dLon * 180/math.pi)
# 	newAlt = original_location.alt + alt
# 	if type(original_location) is LocationGlobal:
# 		targetlocation=LocationGlobal(newlat, newlon, newAlt)
# 	elif type(original_location) is LocationGlobalRelative:
# 		targetlocation=LocationGlobalRelative(newlat, newlon, newAlt)
# 	else:
# 		raise Exception("Invalid Location object passed")
        
# 	return targetlocation;

# #def goto(forward, right, up, gotoFunction=vehicle.simple_goto):
# def goto(forward, right, up):
# 	global vehicle
# 	"""
# 	Moves the vehicle to a position dNorth metres North, dEast metres East and alt meters up of the current position.

# 	The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
# 	the target position. This allows it to be called with different position-setting commands. 
# 	By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

# 	The method reports the distance to target every two seconds.
# 	"""
    
# 	currentLocation = vehicle.location.global_relative_frame

# 	heading = vehicle.heading - 20;
# 	eHeading = heading + 90
# 	if eHeading >= 360:
# 		eHeading -= 360
# 	#print 'Heading: ', heading, '   eHeading: ', eHeading
# 	nSection = int(heading / 90)    #Get which section of the heading is (0 - 3 CW)
# 	#print 'nSection: ', nSection
# 	eSection = nSection + 1 #Get the East direction (0 - 3 CW)
# 	#If the section would be 4 set it to 0
# 	if eSection == 4:
# 		eSection = 0
# 	#print 'eSection: ', eSection

# 	nTheta = 0; #The theta for the north values
# 	eTheta = 0; #The theta for the east values
# 	#Set the nTheta
# 	if nSection == 0:
# 		nTheta = 90 - heading
# 	elif nSection == 1:
# 		nTheta = heading - 90
# 	elif nSection == 2:
# 		nTheta = 270 - heading
# 	else:
# 		nTheta = heading - 270
# 	#Set the eTheta
# 	if eSection == 0:
# 		eTheta = 90 - eHeading
# 	elif eSection == 1:
# 		eTheta = eHeading - 90
# 	elif eSection == 2:
# 		eTheta = 270 - eHeading
# 	else:
# 		eTheta = eHeading - 270

# 	#print 'nTheta: ', nTheta, '   eTheta: ', eTheta

# 	dNorth = 0;
# 	dEast = 0;

# 	if nSection == 0 or nSection == 3:
# 		dNorth = math.sin(math.radians(nTheta)) * forward
# 	else:
# 		dNorth = -math.sin(math.radians(nTheta)) * forward
# 	if nSection == 0 or nSection == 1:
# 		dEast = math.cos(math.radians(nTheta)) * forward
# 	else:
# 		dEast = -math.cos(math.radians(nTheta)) * forward
# 	if eSection == 0 or eSection == 3:
# 		dNorth += math.sin(math.radians(eTheta)) * right
# 	else:
# 		dNorth -= math.sin(math.radians(eTheta)) * right
# 	if eSection == 0 or eSection == 1:
# 		dEast += math.cos(math.radians(eTheta)) * right
# 	else:
# 		dEast -= math.cos(math.radians(eTheta)) * right

# 	#print 'North: ', dNorth, '   East: ', dEast

# 	targetLocation = get_location_metres(currentLocation, dNorth, dEast, up)
# 	#targetDistance = get_distance_metres(currentLocation, targetLocation)
# 	#gotoFunction(targetLocation)
# 	vehicle.simple_goto(targetLocation)

# 	#print "DEBUG: targetLocation: %s" % targetLocation
# 	#print "DEBUG: targetLocation: %s" % targetDistance

# 	"""
# 	while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
# 		#print "DEBUG: mode: %s" % vehicle.mode.name
# 		remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
# 		print("Distance to target: ", remainingDistance)
# 		if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
# 			print("Reached target")
# 			break;
# 		time.sleep(2)
# 	"""

# #connectionFunc()
# #print("Hello")

# print("Hello!")
