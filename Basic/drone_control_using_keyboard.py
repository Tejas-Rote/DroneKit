import pygame
from dronekit import connect,VehicleMode, LocationGlobalRelative,Vehicle
import time 
from pymavlink import mavutil

pygame.init()  
# sets the window title  
pygame.display.set_caption(u'Keyboard events')  
# sets the window size  
s = pygame.display.set_mode((400, 400))  

vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)

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

def send_ned_velocity(Vx, Vy, Vz, duration):
 
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
    for i in range(0,duration):
        vehicle.send_mavlink(msg)
        

    
def condition_yaw(duration, heading, relative=False):
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
        
    for i in range(0,duration):
        # send command to vehicle
        vehicle.send_mavlink(msg)

def Key():
    while True :
        event = pygame.event.wait()  
        if event.type == pygame.QUIT:
            break  
        #--> pressed
        
        if  event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                print("start forward")
             
                send_ned_velocity(40,0,0,5)
            elif event.key == pygame.K_a:
                print("start left")
                send_ned_velocity(0,-40,0,5)
            elif event.key == pygame.K_s:
                print("start backward")
                send_ned_velocity(-40,0,0,5)
            elif event.key == pygame.K_d:
                print("start right ")
                send_ned_velocity(0,40,0,5)
            elif event.key == pygame.K_UP:
                print("star going up ")
                send_ned_velocity(0,0,-40,5)

            elif event.key == pygame.K_LEFT:
                print("start yaw left")
                condition_yaw(5,315,1)
            elif event.key == pygame.K_DOWN:
                print("start going dowm")
                send_ned_velocity(0,0,40,5)

            elif event.key == pygame.K_RIGHT:
                print("start yaw right")
                condition_yaw(5,45,1)


        #--> released 

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w:
                print("stop forward")
                send_ned_velocity(0,0,0,1)
            elif event.key == pygame.K_a:
                print("stop left")
                send_ned_velocity(0,0,0,1)
            elif event.key == pygame.K_s:
                print("stop backward")
                send_ned_velocity(0,0,0,1)
            elif event.key == pygame.K_d:
                print("stop right ")
                send_ned_velocity(0,0,0,1)
            elif event.key == pygame.K_UP:
                print("stop going up ")
                send_ned_velocity(0,0,0,1)

            elif event.key == pygame.K_LEFT:
                print("stop yaw left")
                condition_yaw(5,0,1)

            elif event.key == pygame.K_DOWN:
                print("stop going dowm")
                send_ned_velocity(0,0,0,1)

            elif event.key == pygame.K_RIGHT:
                print("stop yaw right")
                condition_yaw(5,0,1)




arm_and_takeoff(50)
            
Key()
