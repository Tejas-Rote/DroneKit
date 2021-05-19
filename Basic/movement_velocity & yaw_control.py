from dronekit import connect,VehicleMode,LocationGlobalRelative
import time 
from pymavlink import mavutil
vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)

# --> takeoff
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




#---> for velocity component 

def send_ned_velocity(Vx, Vy, Vz, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,  
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        Vx, Vy, Vz, 
        0, 0, 0, 
        0, 0)    

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#---> for yaw component 

def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 
   
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0, #confirmation
        heading,                   # param 1, yaw in degrees
        0,                         # param 2, yaw speed deg/s
        1,                         # param 3, direction -1 ccw, 1 cw
        is_relative,               # param 4, relative offset 1, absolute angle 0
        0, 0, 0)                   # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)



def square():
    N = 40
    E = 40 
    f1 = False 
    f2 = False 
    for i in range(4):
        time.sleep(10)
        if not f1 and not f2: 
            condition_yaw(45)
            send_ned_velocity(N,E,0,5)
            f1 = True 
          
        elif f1 and not f2:
            condition_yaw(135)
            send_ned_velocity(-N,E,0,5)
            f2 = True 
                 
        elif f1 and f2:
            condition_yaw(90)

            send_ned_velocity(-N,-E,0,5)
           
        else:
            condition_yaw(90)
            send_ned_velocity(N,-E,0,5)
         



def main():
    arm_and_takeoff(50)
    square()

if __name__=="__main__":
    main()