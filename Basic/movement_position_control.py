from dronekit import connect,VehicleMode,LocationGlobalRelative
import time


vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)


def moveForward():
    vehicle.mode=VehicleMode("GUIDED")
    location=LocationGlobalRelative(-35.36333366, 149.16527792, 100)
    # Set airspeed using attribute
    vehicle.airspeed = 15 #m/s

    # Set groundspeed using attribute
    vehicle.groundspeed = 7.5 #m/s

    # Set groundspeed using `simple_goto()` parameter
    vehicle.simple_goto(location, groundspeed=10)

# moveForward()

# vehicle.simple_goto()
print(vehicle.home_location)
