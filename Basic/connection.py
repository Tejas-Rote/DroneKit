from dronekit import VehicleMode,  connect 
import time

vehicle = connect('127.0.0.1:14550', wait_ready = True)

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