from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse



def connectMyCopter():

    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    if not connection_string:
        # import dronekit_sitl
        # sitl = dronekit_sitl.start_default()
        # connection_string = sitl.connection_string()
        connection_string = '127.0.0.1:14550'

    vehicle = connect(connection_string, wait_ready = True)

    return vehicle



vehicle = connectMyCopter()
vehicle.mode = VehicleMode('GUIDED')
vehicle.armed = True
vehicle.simple_takeoff(10)

time.sleep(15)

vehicle.mode = VehicleMode('RTL')

print(vehicle.battery)
print(vehicle.system_status)



