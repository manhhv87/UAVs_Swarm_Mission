# This is the main function for follower drone.

# Version 2.0

import time
import numpy as np
# import Queue
from multiprocessing import Queue
import threading
from datetime import datetime

# Step 1: Add this to your PATH environment variables: C:\Program Files (x86)\Windows Kits\10\bin\x64
# Step 2: Copy these files rc.exe & rcdll.dll from C:\Program Files (x86)\Windows Kits\10\bin\10.0.20348.0\x64 to C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\bin
import netifaces as ni
import socket
import dronekit
from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
from dronekit import mavutil
import os
import sys
from MyPythonModule.v4l2_device import Camera
from MyPythonModule import ObjectDetection as od
from MyPythonModule import DroneControlFunction as dcf
# builtins.variables can cross multiple files (for imported functions).
import builtins

############################################################################################################################

# Create global variable to indicate follower status.
builtins.status_waitForCommand = False

# Reserved port.
# The port number should be exactly the same as that in leader drone.
builtins.port_gps = 60001
builtins.port_status = 60002
builtins.port_immediate_command = 60003
builtins.port_heading = 60004

############################################################################################################################

# Caution: This function has to be copied to the main file. Because exec('command', globals()) can only access the variable within the same script.
# This is a server to receive leader drone's command message.
# TO START IT:
# threading.Thread(target=SERVER_receive_and_execute_immediate_command, args=(local_host,)).start()


def SERVER_receive_and_execute_immediate_command(local_host):
    # Create a socket object
    msg_socket = socket.socket()
    msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Bind to the port
    msg_socket.bind((local_host, builtins.port_immediate_command))
    msg_socket.listen(5)                 # Now wait for client connection.
    print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))
    while True:
        # msg_socket.accept() will block while loop until the connection with client is established.
        # Establish connection with client.
        client_connection, client_address = msg_socket.accept()
        print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))
        # Receive message.
        immediate_command_str = client_connection.recv(1024)
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        # If command is 'break', execute immediately, regardless of the status of follower drone.
        if immediate_command_str == 'air_break()':
            # Execute received command.
            exec(immediate_command_str, globals())
            # When command is executed, change status to 'wait for command'.
            builtins.status_waitForCommand = True
            print('{} - Immediate command \'{}\' is finished!'.format(time.ctime(),
                  immediate_command_str))
        # If command is not 'Break', and builtins.status_waitForCommand is true, execute command immediately.
        else:  # immediate_command_str is not 'air_break()'
            if builtins.status_waitForCommand == True:
                # Change builtins.status_waitForCommand to False to block other calls.
                builtins.status_waitForCommand = False
                # Execute immediate command.
                exec(immediate_command_str, globals())
                # Change builtins.status_waitForCommand to True to enable other calls.
                builtins.status_waitForCommand = True
                print(
                    '{} - Immediate command \'{}\' is finished!'.format(time.ctime(), immediate_command_str))
            else:  # builtins.status_waitForCommand == False:
                print('{} - Omit immediate command \'{}\', because builtins.status_waitForCommand is False!'.format(
                    time.ctime(), immediate_command_str))
        # Socket is destroyed when message has been sent.
        client_connection.close()

############################################################################################################################


# Get local host IP.
local_host = ni.ifaddresses('wlan0')[2][0]['addr']
host_specifier = local_host[-1]

# Set log.
flight_log_bufsize = 1  # 0 means unbuffered, 1 means line buffered.
flight_log_filename = 'FlightLog_iris' + host_specifier + \
    '_' + '{:%Y%m%d_%H-%M-%S}'.format(datetime.now()) + '.txt'
flight_log_path = '/home/iris' + host_specifier + '/Log/'
if not os.path.exists(flight_log_path):
    os.makedirs(flight_log_path)
flight_log_path_filename = flight_log_path + flight_log_filename
flight_log = open(flight_log_path_filename, 'w', flight_log_bufsize)
sys.stdout = flight_log

# Specify whether a leader or a follower.
is_leader = False
if is_leader:
    print('{} - This is a leader drone.'.format(time.ctime()))
else:
    print('{} - This is a follower drone.'.format(time.ctime()))

print('{} - local_host = {}.'.format(time.ctime(), local_host))
print('{} - This drone is iris{}'.format(time.ctime(), host_specifier))


# Connect to the Vehicle
print('{} - Connecting to vehicle...'.format(time.ctime()))
vehicle_temp = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)
while not 'vehicle_temp' in locals():
    print('{} - Waiting for vehicle connection...'.format(time.ctime()))
    time.sleep(1)
builtins.vehicle = vehicle_temp
print('{} - Vehicle is connected!'.format(time.ctime()))
# Enable safety switch(take effect after reboot pixhawk).
builtins.vehicle.parameters['BRD_SAFETYENABLE'] = 1  # Enable
# vehicle.parameters['BRD_SAFETYENABLE'] = 0 # Disable

# Start server services.
dcf.start_SERVER_service(builtins.vehicle, local_host)

# Start SERVER_receive_and_execute_immediate_command. NO dcf., this is the newly defined in main.
threading.Thread(
    target=SERVER_receive_and_execute_immediate_command, args=(local_host,)).start()

# Start connection checker. Drone will return home once lost connection.
router_host = '192.168.2.1'
threading.Thread(target=dcf.CHECK_network_connection, args=(
    builtins.vehicle, router_host,), kwargs={'wait_time': 10}).start()

########################################## Initialize follower Balloon Finder ##########################################

# Open camera.
print('\n')
cameraL = Camera('/dev/video0')
cameraR = Camera('/dev/video1')
cameraL.open()
cameraR.open()
# Initialize video device.
resolution = (1920, 1080)
pixelformat = 'MJPG'  # 'YUYV' or 'MJPG'.
exposure = 3
cameraL.init_device(resolution=resolution, pixel_format=pixelformat,
                    exposure=exposure, white_balance='AUTO')
cameraR.init_device(resolution=resolution, pixel_format=pixelformat,
                    exposure=exposure, white_balance='AUTO')
print('\n')
# Initialize memory map.
cameraL.init_mmap()
cameraR.init_mmap()
# Start streaming.
cameraL.stream_on()
cameraR.stream_on()

# Self arm.
print('{} - Self arming...'.format(time.ctime()))
dcf.arm_no_RC(builtins.vehicle)  # Blocking call.
# Once armed, change status_waitForCommand to True.
builtins.status_waitForCommand = True
print('{} - builtins.status_waitForCommand = {}'.format(time.ctime(),
      builtins.status_waitForCommand))
print('{} - Follower is armed!'.format(time.ctime()))
