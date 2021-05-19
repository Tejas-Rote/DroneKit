from dronekit import connect,VehicleMode,LocationGlobal,LocationGlobalRelative,Command
import time 
import math 
from pymavlink import mavutil
vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)

#--> Coordinates 
def get_location_metres(original_location, dNorth, dEast):

    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def adds_square_mission(aLocation, aSize):
    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 50))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 50))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 50))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 50))
    

    print(" Upload new commands to vehicle")
    cmds.upload()

#-- Takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("[INFO] waiting to initialize.....")
        time.sleep(1)
    print("[INFO] arming motors")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True 

    while not vehicle.armed:
        print("[INFO] waiting to arm")
        time.sleep(1)



    print("[INFO] Taking off....")
    vehicle.simple_takeoff(altitude)

    while True:
        print('[INFO] Altitude {}'.format(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= 0.95 * altitude:
            print("[INFO] Target altitude reached ")
            break
        time.sleep(1)

arm_and_takeoff(50)

print('Create a new mission (for current location)')
adds_square_mission(vehicle.location.global_frame,50)


print("Starting mission")
#--> Start Point 
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

print('Return to launch')
vehicle.mode = VehicleMode("RTL")
