import csv
import time

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

# Define connection string and waypoints file
CONNECTION_STRING = "tcp:127.0.0.1:14550"  # Adjust for real drone or SITL
# CONNECTION_STRING = "com6"  # Adjust for real drone or SITL
WAYPOINTS_FILE = "buffered_output.csv"  # Ensure correct path

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect(CONNECTION_STRING,baud=57600, wait_ready=True)
#vehicle = mavutil.mavlink_connection(CONNECTION_STRING,baud=57600)
#vehicle.wait_heartbeat()
print("Vehicle connected.")

def read_waypoints_from_csv(filename):
    """
    Reads waypoints from a CSV file.
    Expected format: latitude, longitude, altitude (in meters)
    """
    waypoints = []
    try:
        with open(filename, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                try:
                    lat, lon, alt = map(float, row)
                    waypoints.append(LocationGlobalRelative(lat, lon, alt))
                except ValueError:
                    print(f"invalid row: {row}")
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        vehicle.close()
        exit(1)
    
    return waypoints

def clear_mission():
    """Clears any existing mission on the drone and flushes old waypoints."""
    print("Clearing old mission...")
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()  # Flush old waypoints
    print("Old mission cleared successfully.")

def upload_mission(waypoints):
    """Uploads a new mission to the drone after ensuring old waypoints are removed."""
    if len(waypoints) < 2:
        print("Error: At least 2 waypoints required.")
        vehicle.close()
        exit(1)

    cmds = vehicle.commands
    cmds.clear()  # Ensure old waypoints are removed before adding new ones

    # Add a takeoff command (only for fixed-wing)
    takeoff_alt = waypoints[0].alt  # Use first waypoint's altitude
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                     waypoints[0].lat, waypoints[0].lon, takeoff_alt))

    # Add waypoints to mission
    for waypoint in waypoints:
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                         waypoint.lat, waypoint.lon, waypoint.alt))

    cmds.upload()  # Upload new mission
    print("New mission uploaded successfully.")

    # Ensure waypoints are properly stored
    print("Verifying mission upload...")
    cmds.download()
    cmds.wait_ready()
    print("Mission waypoints downloaded successfully for verification.")

    # Print downloaded waypoints before initiating flight
    print("**Downloaded Mission Waypoints:**")
    for i, cmd in enumerate(cmds):
        print(f"  Waypoint {i}: lat={cmd.x}, lon={cmd.y}, alt={cmd.z}")
    print("✅ Waypoints verified. Ready for flight.\n")

def wait_for_manual_arm_and_auto():
    """Waits for the vehicle to be armed and in AUTO mode manually."""
    print("Waiting for vehicle to be armed manually...")
    while not vehicle.armed:
        print("Vehicle is not armed. Please arm manually.")
        time.sleep(1)

    print("Vehicle is armed!")

    print("Waiting for AUTO mode to be set manually...")
    while vehicle.mode.name != "AUTO":
        print(f"Current mode: {vehicle.mode.name}. Please switch to AUTO mode.")
        time.sleep(1)

    print("AUTO mode set! Proceeding with mission.")

if __name__ == "__main__":
    waypoints = read_waypoints_from_csv(WAYPOINTS_FILE)

    clear_mission()  # Clear old mission before uploading
    upload_mission(waypoints)  # Upload new mission

    # Print waypoints before flight initiation
    print("Mission setup complete. Review waypoints above before arming the drone.**\n")

    # Wait for user to arm the vehicle and set AUTO mode manually
    wait_for_manual_arm_and_auto()

    print("Mission execution started...")

    while vehicle.mode.name == "AUTO":
        print(f"Current WP: {vehicle.commands.next}, Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        time.sleep(2)

    print("Mission completed or mode changed manually.")

    # Close vehicle connection
    vehicle.close()
 