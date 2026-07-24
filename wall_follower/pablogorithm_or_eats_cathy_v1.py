"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here

speed = 0
angle = 0
right_dis = 0.0
right_max_distance_angle = 0
left_dis = 0.0
left_max_distance_angle = 0


########################################################################################
# Functions
########################################################################################

def right_max_dis(scan):
    max_avg = -1
    max_dis_angle = -1
    samples = rc.lidar.get_num_samples()

    for i in range(0, 180):# 720
        avg = rc_utils.get_lidar_average_distance(scan, i/2 , 20)
        if avg > max_avg:
            max_avg = avg
            max_dis_angle = i/2
    
    return max_dis_angle, max_avg


def left_max_dis(scan):
    max_avg = -1
    max_dis_angle = -1
    samples = rc.lidar.get_num_samples()

    for i in range(719, 540, -1):# (3*samples)/4
        avg = rc_utils.get_lidar_average_distance(scan, i/2 , 20)
        if avg > max_avg:   
            max_avg = avg
            max_dis_angle = i/2
    
    return max_dis_angle, max_avg


# [FUNCTION] The start function is run once every time the start button is pressed
def start():

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global angle
    global speed
    global right_dis
    global right_max_distance_angle
    global left_dis
    global left_max_distance_angle

    scan = rc.lidar.get_samples()

    right_max_distance_angle, right_dis = right_max_dis(scan)
    left_max_distance_angle, left_dis = left_max_dis(scan)

    error = right_dis - left_dis # used to weigh the importance of a distance or angle based on distance(longer -> more importance)

    kp = 0.002125

    angle = kp * error

    angle = rc_utils.clamp(angle, -1, 1)

    speed = 1

    # maybe include a speed controller when speed is higher

    print(f"right distance: {right_dis} | left distance: {left_dis}")
    print(f"right max distance angle: {right_max_distance_angle} | left max distance angle: {left_max_distance_angle}")
    print(f"error: {error} | angle: {angle} | speed: {speed}")

    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
