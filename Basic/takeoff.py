from dronekit import connect,VehicleMode,LocationGlobalRelative,LocationGlobal
import time
vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)

def arm_and_takeoff(altitude):
    print("basic checkup")
    while not vehicle.is_armable:
        print("waiting to initialize")
        time.sleep(1)
    print("arming motors")
    vehicle.mode=VehicleMode("GUIDED")
    vehicle.armed=True

    while not vehicle.armed:
        print("waiting to be armed")
        time.sleep(1)
    print(vehicle.home_location)
    print("taking off")
    vehicle.simple_takeoff(altitude)

    while True:
        print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
        
        if vehicle.location.global_relative_frame.alt>=altitude*0.95:
            print("target altitude reached")
            break
        time.sleep(1)

arm_and_takeoff(50)

vehicle.gimbal.rotate(-90, 0, 0)

# vehicle.mode=VehicleMode("GUIDED")
# vehicle.armed=True
# how to get home location

vehicle.airspeed = 15 #m/s

# Set groundspeed using attribute
vehicle.groundspeed = 15 #m/s
