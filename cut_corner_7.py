import math
import pandas as pd
import matplotlib
# matplotlib.use('Agg')  # Use a safe, file-only backend
import matplotlib.pyplot as plt
from dronekit import connect, Command
from pymavlink import mavutil

# === CONFIGURATION ===
INPUT_CSV = "test waypoints(Sheet1).csv"
CONNECTION_STRING = "tcp:127.0.0.1:14550"  # Or COM3, etc.
ANGLE_THRESHOLD = 130
FWD_DIST = 0.0001
LAT_LON_OFFSET = -0.0001
ALT_OFFSET = 2.0

# --- Path Processing Functions --- 
def detect_sharp_corners_2d(waypoints_3d, angle_threshold_deg):
    sharp_corners = []
    for i in range(1, len(waypoints_3d) - 1):
        lat_in, lon_in, _ = waypoints_3d[i - 1]
        lat_c, lon_c, _ = waypoints_3d[i]
        lat_out, lon_out, _ = waypoints_3d[i + 1]
        vx_in = lon_c - lon_in
        vy_in = lat_c - lat_in
        vx_out = lon_out - lon_c
        vy_out = lat_out - lat_c

        len_in = math.hypot(vx_in, vy_in)
        len_out = math.hypot(vx_out, vy_out)
        if len_in < 1e-9 or len_out < 1e-9:
            continue

        ux_in, uy_in = vx_in / len_in, vy_in / len_in
        ux_out, uy_out = vx_out / len_out, vy_out / len_out
        dot_val = max(-1.0, min(1.0, ux_in * ux_out + uy_in * uy_out))
        angle_deg = math.degrees(math.acos(dot_val))

        if angle_deg < angle_threshold_deg:
            sharp_corners.append(i)
    return sharp_corners

def offset_two_points(w_in, w_corner, w_out, fwd_dist, lat_lon_offset, alt_offset):
    (lat_in, lon_in, _) = w_in
    (lat_c, lon_c, alt_c) = w_corner
    (lat_out, lon_out, _) = w_out

    vx_in = lon_c - lon_in
    vy_in = lat_c - lat_in
    len_in = math.hypot(vx_in, vy_in)
    ux_in, uy_in = (vx_in / len_in, vy_in / len_in) if len_in > 0 else (0, 1)

    vx_out = lon_out - lon_c
    vy_out = lat_out - lat_c
    len_out = math.hypot(vx_out, vy_out)
    ux_out, uy_out = (vx_out / len_out, vy_out / len_out) if len_out > 0 else (0, 1)

    cross = vx_in * vy_out - vy_in * vx_out
    perp_x, perp_y = (-uy_in, ux_in) if cross > 0 else (uy_in, -ux_in)

    p1_lon = lon_c - fwd_dist * ux_in + lat_lon_offset * perp_x
    p1_lat = lat_c - fwd_dist * uy_in + lat_lon_offset * perp_y
    p1_alt = alt_c + alt_offset

    p2_lon = lon_c + fwd_dist * ux_out + lat_lon_offset * perp_x
    p2_lat = lat_c + fwd_dist * uy_out + lat_lon_offset * perp_y
    p2_alt = alt_c + alt_offset

    return (p1_lat, p1_lon, p1_alt), (p2_lat, p2_lon, p2_alt)

def insert_two_points_at_corners(waypoints_3d, corner_indices,
                                 fwd_dist, lat_lon_offset, alt_offset):
    new_path = []
    original_corners = []
    corner_set = set(corner_indices)
    n = len(waypoints_3d)
    i = 0
    while i < n:
        if i not in corner_set:
            new_path.append(waypoints_3d[i])
            i += 1
        elif 0 < i < n - 1:
            w_in = waypoints_3d[i - 1]
            w_corner = waypoints_3d[i]
            w_out = waypoints_3d[i + 1]
            p1, p2 = offset_two_points(w_in, w_corner, w_out, fwd_dist, lat_lon_offset, alt_offset)
            new_path.append(p1)
            new_path.append(p2)
            original_corners.append(w_corner)
            i += 1
        else:
            new_path.append(waypoints_3d[i])
            i += 1
    return new_path, original_corners

def plot_paths(original, adjusted, corners):
    orig_lats = [pt[0] for pt in original]
    orig_lons = [pt[1] for pt in original]
    adj_lats = [pt[0] for pt in adjusted]
    adj_lons = [pt[1] for pt in adjusted]
    corner_lats = [pt[0] for pt in corners]
    corner_lons = [pt[1] for pt in corners]

    plt.figure(figsize=(10, 6))
    plt.plot(orig_lons, orig_lats, 'r--', label="Original Path")
    plt.plot(adj_lons, adj_lats, 'b-', label="Smoothed Path")
    plt.scatter(corner_lons, corner_lats, c='green', marker='o', s=100, label="Sharp Corners")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Waypoint Smoothing Visualization")
    plt.legend()
    plt.grid(True)
    plt.show()

# --- DroneKit Upload Functions --- 
def connect_vehicle(connection_string):
    print(f"Connecting to vehicle on {connection_string}...")
    vehicle = connect(connection_string, wait_ready=True)
    print("✅ Connected to vehicle.")
    return vehicle

def upload_mission(vehicle, waypoint_list):
    cmds = vehicle.commands
    cmds.clear()

    cmds.add(Command(0, 0, 0,
                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0,
                     waypoint_list[0][0], waypoint_list[0][1], waypoint_list[0][2]))

    for lat, lon, alt in waypoint_list[1:]:
        cmds.add(Command(0, 0, 0,
                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0,
                         lat, lon, alt))

    cmds.upload()
    print("✅ Mission uploaded successfully.")

# --- Main Execution ---

def main():
    df = pd.read_csv(INPUT_CSV)
    waypoints = list(zip(df['lat'], df['lon'], df['alt']))

    print("Detecting sharp corners...")
    corner_indices = detect_sharp_corners_2d(waypoints, ANGLE_THRESHOLD)

    print("Inserting buffer waypoints...")
    new_path, corners = insert_two_points_at_corners(waypoints, corner_indices, FWD_DIST, LAT_LON_OFFSET, ALT_OFFSET)

    print("Plotting path...")
    plot_paths(waypoints, new_path, corners)

    print("Connecting and uploading mission...")
    vehicle = connect_vehicle(CONNECTION_STRING)
    upload_mission(vehicle, new_path)

if __name__ == "__main__":
    main()
