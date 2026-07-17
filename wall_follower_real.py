"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

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

import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
speed = 0.0
angle = 0.0
left_forward_distance = 0.0
right_forward_distance = 0.0
kD = 0.0
kP = -0.003 #fine???? idk
last_error = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
   global speed
   global angle


   # Initialize variables
   speed = 0
   angle = 0


   # Set initial driving speed and angle
   rc.drive.set_speed_angle(speed, angle)


   # Set update_slow to refresh every half second
   rc.set_update_slow_time(0.5) # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global left_forward_distance
    global right_forward_distance
    global last_error

    rc.drive.set_max_speed(1)


    speed = 1
    scan = rc.lidar.get_samples()
    right_forward_distance = rc_utils.get_lidar_average_distance(scan, 405, 20)
    left_forward_distance = rc_utils.get_lidar_average_distance(scan, 675, 20)

    #present_value = 0
    #setpoint = 0
    #kp = 0.01

    setpoint = (left_forward_distance + right_forward_distance)//2
    present_value = right_forward_distance

    error = setpoint-present_value
    angle = kP * error + kD * (error - last_error)/rc.get_delta_time()
    angle = rc_utils.clamp(angle, -1, 1)
    last_error = error

    """
    if left_forward_distance==0:
        angle = -0.659
    elif right_forward_distance==0:
        angle = 0.659
    """
    

    #print("angle", angle, "setpoint", setpoint, "farthest distance", present_value, "forward distance", forward_distance)

    rc.drive.set_speed_angle(speed, angle)

        




# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    print("angle", angle, "left forward", left_forward_distance, "right forward", right_forward_distance)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
