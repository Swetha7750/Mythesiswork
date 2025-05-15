import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import struct
import qtm
import math
import socket
import json
from pymavlink import mavutil
import time
import os
import pandas as pd
import numpy as np
import signal
import matplotlib.pyplot as plt
import csv
#import scipy
# import scip
from scipy.spatial.transform import Rotation as R

# === CONFIG ===
wanted_body = "drone_body"
QTM_IP = "192.168.252.1"  # Your QTM server IP
PASSWORD = "password"
RADIUS_OF_EARTH = 6371000.0
out_addr = ("127.0.0.1", 25100)
latitude_0 = 50.764905
longitude_0 = 6.07875

vehicle = None
armed = False
# PID parameters
X_kp, x_ki, x_kd = 0.25, 0.0, -0.3
y_kp, y_ki, y_kd = 0.07, 0.0, -0.13
ts = 0.02  # time step = 50 Hz
time_data = []
x_cmd_data = []
XX_data = []

y_cmd_data =[]
yy_data = []

theta_cmd_data =[]
theta_radians =[]


phi_cmd_data=[]


pwmtheta_data =[]
pwmphi_data =[]
roll_angle_data =[]
pitch_angle_data=[]
yaw_angle_data =[]

captured = False  # To track if values have been captured
released = False  # To track if values have been reset to 0
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"  # Replace with your MAVLink connection string

async def set_rc_channel_pwm(channel_id, pwm):
    if channel_id < 1 or channel_id >18:
        print('sorry the channel doesnt exist')
        return
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    vehicle.mav.rc_channels_override_send(
        vehicle.target_system,                # target_system
        vehicle.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

async def arm_vehicle():
    """ Arm the vehicle """
    global armed
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        armed = True
        print("Vehicle armed!")
    else:
        print("Failed to arm vehicle!")


async def disarm_vehicle():
    """ Disarm the vehicle """
    global armed
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        armed = False
        print("Vehicle disarmed!")
    else:
        print("Failed to disarm vehicle!")

async def connect_mavlink():
    """ Connect to the vehicle using MAVLink """
    global vehicle
    try:
        vehicle = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        vehicle.wait_heartbeat()
        print("Connected to vehicle!")
    except Exception as e:
        print(f"Failed to connect to vehicle: {e}")
        raise

# Signal handler to handle Ctrl+C and save the data
def signal_handler(sig, frame):
    print("Ctrl+C pressed, saving data...")
    save_data()
    #save_data_theta()
    print("Data saved. Exiting program.")
    exit(0)

# Save data to CSV when Ctrl+C is pressed
def save_data():
   # assert len(set(map(len, [time_data, x_cmd_data, XX_data, y_cmd_data, yy_data, theta_cmd_data, phi_cmd_data, pwmtheta_data, pwmphi_data]))) == 1,"All data lists must be the same length
    with open('trajectory_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'x_cmd', 'XX', 'y_cmd', 'YY', 'theta_cmd', 'phi_cmd', 'pwm_theta', 'pwm_phi', 'yaw', 'roll', 'pitch'])  # Column headers
        for t, x_cmd, XX, y_cmd, YY, theta_cmd, phi_cmd, pwm_theta, pwm_phi, yaw, roll, pitch  in zip(time_data, x_cmd_data, XX_data, y_cmd_data, yy_data, theta_cmd_data, phi_cmd_data, pwmtheta_data, pwmphi_data, yaw_angle_data, roll_angle_data, pitch_angle_data):# y_cmd_data, yy_data):
            writer.writerow([t, x_cmd, XX, y_cmd, YY, theta_cmd, phi_cmd, pwm_theta, pwm_phi, yaw, roll, pitch ])#y_cmd, YY])

def save_data_theta():
    with open('trajectory_data_theta.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Time', 'theta'])  # Column headers
        for t, theta in zip(time_data, theta_cmd_data):# y_cmd_data, yy_data):
            writer.writerow([t, theta] )#y_cmd, YY])





# Shared drone position (updated from QTM)
XX, YY = 0.0, 0.0
x_integral = 0
y_integral = 0 



# Socket setup
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

frozen = False 
async def set_rc_channel_pwm(channel_id, pwm):
    if channel_id < 1 or channel_id >18:
        print('sorry the channel doesnt exist')
        return
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    vehicle.mav.rc_channels_override_send(
        vehicle.target_system,                # target_system
        vehicle.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.
    
async def arm_vehicle():
    """ Arm the vehicle """
    global armed
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        armed = True
        print("Vehicle armed!")
    else:
        print("Failed to arm vehicle!")


async def disarm_vehicle():
    """ Disarm the vehicle """
    global armed
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        armed = False
        print("Vehicle disarmed!")
    else:
        print("Failed to disarm vehicle!")


async def connect_mavlink():
    """ Connect to the vehicle using MAVLink """
    global vehicle
    try:
        vehicle = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        vehicle.wait_heartbeat()
        print("Connected to vehicle!")
    except Exception as e:
        print(f"Failed to connect to vehicle: {e}")
        raise


# === PID Trajectory Generator ===
async def straight_line():
    global captured, released, x_cmd, y_cmd, capture_time, XX, YY, theta, theta_cmd, pwm_theta, pwm_phi
    global x_integral, y_integral, XX_prev, YY_prev

    x_cmd = 0.0
    y_cmd = 0.0
    capture_time = 0.0
    XX_prev = XX
    YY_prev = YY

    duration = 300.0  # Total run time
    # frequency = 50    # Hz
    # dt = 1 / frequency

    print("Starting PID trajectory control...")
    start_time = time.time()

    while time.time() - start_time < duration:
        current_time = time.time() - start_time

        # Lock the command values at t=30
        if not captured and current_time >= 30:
            x_cmd = XX
            y_cmd = YY
            capture_time = current_time
            captured = True
            
        # Reset to 0 after 60 seconds
        elif captured and not released and current_time >= capture_time + 60:
            x_cmd = 0
            y_cmd = 0
            released = True
            
       
       
        
        # --- X Axis ---
        X_error = x_cmd - XX
        x_integral += X_error*ts
        x_integral = max(min(x_integral, 2), -2)
        X_dot = (XX-XX_prev)/ts
        xp = X_kp * X_error
        xi = x_ki * x_integral
        xd = x_kd * X_dot
        XX_prev = XX  # previous position 

        max_theta = 5
        min_theta = -5
        theta = (xp + xi + xd)
        theta = max(min(theta, max_theta), min_theta)
        theta_cmd = theta * (180 / math.pi)

        # --- Y Axis ---
        Y_error = y_cmd - YY
        y_integral += Y_error*ts
        y_integral = max(min(y_integral, 2), -2)  # Anti-windup
        Y_dot = (YY-YY_prev)/ts
        yp = y_kp * Y_error
        yi = y_ki * y_integral
        yd = y_kd * Y_dot
        YY_prev = YY

        phi = (yp + yi + yd)
        phi = max(min(phi, 5), -5)
        phi_cmd = phi * (180 / math.pi)
        
        


        min_angle=-30
        max_angle =30
        min_pwm =1000
        max_pwm =2000

        # threshold_max= 1700
        # threshold_min = 1300

        #for theta
        theta_cmd= max(min(theta_cmd, max_angle), min_angle)
        normalized = (theta_cmd-min_angle)/(max_angle-min_angle)
        pwm_theta=int(min_pwm+normalized*(max_pwm-min_pwm)) # pwm for theta
        pwm_theta=max(min(pwm_theta, max_pwm),min_pwm) # saturating 


        #for phi
        phi_cmd= max(min(phi_cmd, max_angle), min_angle)
        normalized = (phi_cmd-min_angle)/(max_angle-min_angle)  #normalizing the value between 0-1
        pwm_phi=int(min_pwm+normalized*(max_pwm-min_pwm)) # calculating the pwm value for roll
        pwm_phi=max(min(pwm_phi, max_pwm),min_pwm)


        
        await set_rc_channel_pwm(2, pwm_phi)
        await set_rc_channel_pwm(3, pwm_theta)
        
        
        print(f"t={current_time:.2f}s, x_cmd={x_cmd:.2f}, y_cmd={y_cmd:.2f}, XX:{XX:.2f}, YY:{YY:.2f}, x_error:{X_error:.2f}, y_error:{Y_error:.2f}")
        print(f"  -> theta_cmd: {theta:.3f}, phi_cmd: {phi:.3f}, pwm_theta:{pwm_theta}, pwm_phi:{pwm_phi}, theta_degree:{theta_cmd:.2f}, phi_degree:{phi_cmd:.2f}")# roll_angle:{euler_angles:2f}")# roll{roll_degrees:.2f}")
        print(f"  -> roll_angle: {roll:.2f}, pitch_angle: {pitch:.2f}, yaw_angle: {yaw:.2f}")
        time_data.append(current_time)
        x_cmd_data.append(x_cmd)
        XX_data.append(XX)
        y_cmd_data.append(y_cmd)
        yy_data.append(YY)


        theta_cmd_data.append(theta_cmd)
        phi_cmd_data.append(phi_cmd)

        pwmtheta_data.append(pwm_theta)
        pwmphi_data.append(pwm_phi)
        roll_angle_data.append(roll)
        pitch_angle_data.append(pitch)
        yaw_angle_data.append(yaw)
        
        await asyncio.sleep(0.02)


# === Create mapping from body name to index ===
def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index
    return body_to_index

# === QTM Streaming Callback ===
def on_packet(packet):
    global XX, YY,roll, pitch, yaw

    info, bodies = packet.get_6d()
    if wanted_body not in body_index:
        return

    idx = body_index[wanted_body]
    pos, _ = bodies[idx]
    
    position, rotation = bodies[idx]
    rotation_matrix = np.array(rotation).reshape(3, 3)
    r = R.from_matrix(rotation_matrix.T)
    euler_angles = r.as_euler('xyz', degrees=True)
    roll, pitch, yaw = euler_angles  # assuming XYZ order
    # trans_rotation_matrix = rotation_matrix.T
    # roll = math.atan2(trans_rotation_matrix[2][1], trans_rotation_matrix[2][2])  # Correct usage of atan2

    #         # Convert roll from radians to degrees if needed
    # roll_degrees = math.degrees(roll)


    if math.isnan(pos.x):
        return

    # Convert mm to meters
    XX = (round(pos.x, 5) / 1000.0)
    YY = (round(pos.y, 5) / 1000.0)
    ZZ = round(pos.z, 5)

    # Convert local to GPS
    lat_rad = math.radians(latitude_0)
    lat = latitude_0 + (YY / RADIUS_OF_EARTH) * (180 / math.pi)
    lon = longitude_0 + (XX / (RADIUS_OF_EARTH * math.cos(lat_rad))) * (180 / math.pi)
    #rotation_matrix = np.array(rotation).reshape(3, 3)
    # Send GPS-like UDP packet
    gps_data = {
        'time_usec': 0,
        'gps_id': 0,
        'ignore_flags': 8,
        'time_week_ms': 0,
        'time_week': 0,
        'fix_type': 3,
        'lat': int(lat * 1e7),
        'lon': int(lon * 1e7),
        'alt': ZZ,
        'hdop': 1,
        'vdop': 1,
        'vn': 0,
        've': 0,
        'vd': 0,
        'speed_accuracy': 0.5,
        'horiz_accuracy': 1.0,
        'vert_accuracy': 1.5,
        'satellites_visible': 9
    }

    out_data = json.dumps(gps_data)
    s.sendto(out_data.encode(), out_addr)

# === Main Asynchronous Function ===
async def main():
    global body_index

    # Connect to QTM
    connection = await qtm.connect(QTM_IP)
    if connection is None:
        print("Failed to connect to QTM.")
        return

    # Take control of QTM
    async with qtm.TakeControl(connection, PASSWORD):
        await connection.new()  # Start real-time measurement

    # Get body index mapping
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    # Run QTM streaming and PID control concurrently
    await asyncio.gather(
        connection.stream_frames(components=["6d"], on_packet=on_packet),
       
    ) 
    await connect_mavlink()
    await arm_vehicle()
    await straight_line()
    await disarm_vehicle()


# === Run Script ===
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    asyncio.run(main())
