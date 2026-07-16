"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: Cathy Lyu

Purpose: This script provides the RACECAR with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the RACECAR should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The RACECAR may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a 
Python file in the RACECAR sim. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: Test your code using the level "Neo Labs > Lab I: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

angle = 0
speed = 0
time_delta = 0
turning = False
left_forward = -1
right_forward = -1
last_error = 0
# Declare any global variables here


########################################################################################
# Functions
########################################################################################
def clamp(value: float, min: float, max: float) -> float:
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value
    
def wall_following():
    global last_error
    global speed
    global angle
    global left_forward
    global right_forward

    scan = rc.lidar.get_samples()
    #left = scan[540]
    #right = scan[180]
    #forward = scan[0]
    # left_forward = scan[640]
    # right_forward = scan[80]
    left_forward = rc_utils.get_lidar_average_distance(scan, 640, 10)
    right_forward = rc_utils.get_lidar_average_distance(scan, 80, 10)
    # print(f"left: {left} | right: {right} | forward: {forward}")

    error = left_forward - right_forward
    # error_x = left - right

    # error = left - right

    speed = 1
    kp = -0.0008
    kd = 0
    # kp = -0.01324
    angle = error * kp + kd * (error - last_error)/rc.get_delta_time()
    angle = clamp(angle, -1, 1)
    last_error = error
    # print(f"angle: {angle}")

    # if turning:
    #     time_delta += rc.get_delta_time()
    #     if time_delta > 0.6:
    #         turning = False
    # if forward < 100:
    #     return

    # if left_forward <= 27:
    #     angle = -0.99
    #     speed = 0.8
    # elif right_forward <= 27:
    #     angle = 0.99 # 0.99
    #     speed = 0.8
    # elif left + right >= 100:
    #     speed = 1
    #     if abs(error) < 35:
    #         angle = 0
    # elif left + right < 100:
    #     rc.drive.set_max_speed(0.27)
    #     speed = 0.86
        # if abs(error) < 2:
        #     angle = 0
    

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    rc.drive.set_max_speed(1)


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global time_delta
    
    wall_following()
    rc.drive.set_speed_angle(speed, angle)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global left_forward
    global right_forward
    global angle
    
    print(f"left_forward: {left_forward} | right_forward: {right_forward}")
    print(f"angle: {angle}")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
