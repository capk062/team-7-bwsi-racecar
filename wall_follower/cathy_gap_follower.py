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
import numpy as np

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

########################################################################################
# Functions
##########/////////////////////##############################################################################

def gap_detection(scan):
    samples = rc.lidar.get_num_samples()
    # scan = rc.lidar.get_samples()
    # filtered_scan = np.full(1080, -1)
    index_left = -1.0
    index_right = -1.0

    for i in range(0, 301):# reality: range(0, 301) - total:1080 | sim: total-720 range(0, 201)
        if scan[i] > 125:
            # filtered_scan[i] = 0
            index_right = i

    for i in range(1079, 779, -1):# reality: range(1079, 779, -1) - total:1080 | sim: total-720 range(719, 519, -1) * not including 519 and 720 doesn't exist
        # print(f"scan angle distances: {scan[i]}")
        if scan[i] > 125:
            # filtered_scan[i] = 0
            index_left = i - 1080 # reality: i - 1080 | sim: i - 720

    mid_pos = (((index_left + index_right)/3)/2)/90 # /3 to turn to angle and /2 for angle midpoint /90

    return index_left, index_right, mid_pos


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    rc.drive.set_speed_angle(0, 0)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle

    scan = rc.lidar.get_samples()

    gap_left, gap_right, angle = gap_detection(scan)

    angle = rc_utils.clamp(angle, -1, 1)
    speed = 1

    print(f"left: {gap_left} | right: {gap_right} | angle: {angle}")

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