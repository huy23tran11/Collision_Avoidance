from pynput import keyboard
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

FLIGHT_ALT = 10
ACTION = "NONE"
STATUS = "Initializing Program"
MODE = "NONE"
vehicle = None
targetLoc = None
homeLocation = None

def printStatus():
    global MODE
    MODE = vehicle.mode.name
    print(MODE, ACTION, STATUS)

def on_press(key):
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in ['s']:  # keys of interest
        stop()
    if k in ['t']:  # keys of interest
        arm_and_takeoff(FLIGHT_ALT)
    if k in ['g']:  # keys of interest
        goToTargetLoc()
    if k in ['i']:  # keys of interest
        printStatus()
    if k in ['1']:  # keys of interest
        goto_position_target_body_ned(0, 10, 0)
        
        
def arm_and_takeoff(aTargetAltitude):
    global ACTION
    global STATUS
    ACTION = "Auto Takeoff"
    STATUS = "Attempting Auto Take Off"
    printStatus()
    time.sleep(10)
    
    while not vehicle.is_armable:
        STATUS = "Waiting for vehicle to initialise..."
        printStatus()
        time.sleep(10)

    STATUS = "Arming motors"
    vehicle.mode    = VehicleMode("GUIDED")
    printStatus()
    vehicle.armed = True

    while not vehicle.armed:      
        STATUS = " Waiting for arming..."
        printStatus()
        time.sleep(3)

    STATUS = "Attempting Auto Take Off in 8 seconds"
    printStatus()
    time.sleep(8) # not working because armming too long
    STATUS = "Taking off!"
    printStatus()
    vehicle.simple_takeoff(aTargetAltitude)
    
    while vehicle.location.global_relative_frame.alt < aTargetAltitude*0.95:
        STATUS = "CLIMBING"
        printStatus()
        time.sleep(1)

    STATUS = 'Took Off, Awaiting Commands'
    printStatus()


def setTargetLoc():
    global targetLoc
    targetLoc = LocationGlobalRelative(34.043349, -117.811809, FLIGHT_ALT)
    # targetLoc = get_location_metres(vehicle.location.global_relative_frame, 100, 100)
    print("Target Location: ", targetLoc.lat, " (lat), ", targetLoc.lon, " (long)")
    time.sleep(1) # sleep for the interface to work  if not sleep the function was called in cpp but not executed somehow 
        
def stop():
    global ACTION
    global STATUS
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
    ACTION = "STOPPING"
    STATUS = "STOPPING"
    printStatus()
    vehicle.send_mavlink(msg)
    time.sleep(1) # sleep for the interface to work  if not sleep the function was called in cpp but not executed somehow 
    
def goToTargetLoc():
    global targetLoc
    print("Running")
    global ACTION
    global STATUS
    ACTION = "GOING TO TARGET"
    STATUS = "GOING TO TARGET"
    printStatus()
    vehicle.simple_goto(targetLoc, 1, 1)
    print("going to target loc")
    time.sleep(1) # sleep for the interface to work  if not sleep the function was called in cpp but not executed somehow 

        
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth  
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto_position_target_body_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    set_roi(targetLoc)

def slide_left(): #slide left 0.1 m at a time
    goto_position_target_body_ned(0, -0.5, 0); 

def slide_right(): #slide left 0.1 m at a time
    goto_position_target_body_ned(0, 0.5, 0); 

def set_roi(location):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)

def connectionFunc():
    global vehicle
    connection_string = 'tcp:192.168.86.182:5763'
    # connection_string = '/dev/ttyACM0, 57600'
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

def setLocation():
    global homeLocation
    homeLocation = vehicle.location.global_relative_frame
    print("Home Location: ", homeLocation.lat, " (lat), ", homeLocation.lon, " (long)")

# connectionFunc()
# setLocation()
# arm_and_takeoff(10)
# setTargetLoc()
# goToTargetLoc()
# connection_string = 'tcp:192.168.86.182:5763'
# vehicle = connect(connection_string, wait_ready=True)
# homeLocation = vehicle.location.global_relative_frame
# print("Home Location: ", homeLocation.lat, " (lat), ", homeLocation.lon, " (long)")
# listener = keyboard.Listener(on_press=on_press)
# listener.start()  # start to listen on a separate thread
# listener.join()