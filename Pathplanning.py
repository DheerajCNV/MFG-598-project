import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import math
from collections.abc import MutableMapping  # Import MutableMapping from collections.abc

# Function to calculate Ground Sample Distance (GSD)
def calculate_gsd(sensor_width, image_width, flight_height, focal_length):
    gsd = (sensor_width * flight_height) / (focal_length * image_width)
    return gsd

# Function to generate optimized waypoints for UAV path planning
def optimized_uav_path(terrain_width, terrain_height, flight_height, overlap_percentage, sensor_width, image_width, image_height, focal_length):
    gsd = calculate_gsd(sensor_width, image_width, flight_height, focal_length)
    footprint_width = gsd * image_width
    footprint_height = gsd * image_height

    # Determine primary flight direction based on terrain aspect ratio
    if terrain_width > terrain_height:
        primary_direction = 'height'
        path_spacing = footprint_height * (1 - overlap_percentage / 100)
        image_spacing = footprint_width * (1 - overlap_percentage / 100)
        terrain_long, terrain_short = terrain_width, terrain_height
    else:
        primary_direction = 'width'
        path_spacing = footprint_width * (1 - overlap_percentage / 100)
        image_spacing = footprint_height * (1 - overlap_percentage / 100)
        terrain_long, terrain_short = terrain_height, terrain_width

    # Calculate the number of paths and number of images per path for optimal coverage
    num_paths = int(terrain_short / path_spacing) + 1
    images_per_path = int(terrain_long / image_spacing) + 1

    # Generate waypoints in an efficient manner
    waypoints = []
    for path in range(num_paths):
        if path % 2 == 0:  # Even paths go in one direction
            for image_num in range(images_per_path):
                if primary_direction == 'width':
                    x = image_num * image_spacing
                    y = path * path_spacing
                else:
                    x = path * path_spacing
                    y = image_num * image_spacing
                waypoints.append((x, y, flight_height))
        else:  # Odd paths return, reducing unnecessary travel
            for image_num in reversed(range(images_per_path)):
                if primary_direction == 'width':
                    x = image_num * image_spacing
                    y = path * path_spacing
                else:
                    x = path * path_spacing
                    y = image_num * image_spacing
                waypoints.append((x, y, flight_height))

    return waypoints


# Function to connect to the drone and execute path planning
def connect_and_execute_path_planning():
    # Get user inputs
    connection_string = input("Enter connection string: ")
    terrain_width = float(input("Enter terrain width (in meters): "))
    terrain_height = float(input("Enter terrain height (in meters): "))
    flight_height = float(input("Enter flight height (in meters): "))
    overlap_percentage = float(input("Enter overlap percentage: "))
    sensor_width = float(input("Enter sensor width (in mm): "))
    image_width = int(input("Enter image width (in pixels): "))
    image_height = int(input("Enter image height (in pixels): "))
    focal_length = float(input("Enter focal length (in mm): "))

    # Connect to the vehicle
    print("Connecting to vehicle...")
    vehicle = connect(connection_string, wait_ready=True)
    print("Connected!")

    # Generate optimized waypoints
    print("Generating optimized waypoints...")
    optimized_waypoints = optimized_uav_path(terrain_width, terrain_height, flight_height, overlap_percentage, sensor_width, image_width, image_height, focal_length)
    print("Optimized waypoints generated!")

    # Arm and take off
    print("Arming and taking off...")
    arm_and_takeoff(vehicle, flight_height)
    print("Armed and taken off!")

    # Send drone to each waypoint
    print("Sending drone to each waypoint...")
    for i, waypoint in enumerate(optimized_waypoints):
        print(f"Moving to waypoint {i + 1}/{len(optimized_waypoints)}...")
        goto(vehicle, waypoint[0], waypoint[1], waypoint[2])
    print("All waypoints reached!")

    # Land and close connection
    print("Landing and closing connection...")
    land_and_close(vehicle)
    print("Landed and connection closed!")


# Function to arm and take off the drone
def arm_and_takeoff(vehicle, target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Function to navigate the drone to a specific GPS location
def goto(vehicle, north, east, altitude):
    target_location = LocationGlobalRelative(north, east, altitude)
    vehicle.simple_goto(target_location)
    while True:
        distance_to_target = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        if distance_to_target <= 1:  # Adjust this threshold as needed
            print("Reached target waypoint")
            break
        time.sleep(1)


# Function to calculate distance between two GPS locations
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# Function to land the drone and close connection
def land_and_close(vehicle):
    print("Setting RTL mode...")
    vehicle.mode = VehicleMode("RTL")
    print("Close vehicle object")
    vehicle.close()

# Main function
if __name__ == "__main__":
    print("Welcome to the UAV Path Planning System!")
    connect_and_execute_path_planning()
    print("Completed")