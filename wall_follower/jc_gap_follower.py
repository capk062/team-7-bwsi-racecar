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
import numpy as np

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
speed = 0.0
angle = 0.0
kD = -0.004 #-0.005
kP = -0.02
last_error = 0
LOOK_AHEAD_DISTANCE = 125

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def gap_follow():
    scan = rc.lidar.get_samples()
    distances = np.zeros(600)
    angles = np.zeros(600)
    index = 0
    angle = -300
    curr_zero_gap_length = 0
    start_gap = -1
    max_start_gap = -1
    max_zero_gap_length = 0
    middle_angle = -10000
    for i in range(780, 1080):
        if scan[i]<=LOOK_AHEAD_DISTANCE and scan[i]!=0:
            distances[index] = scan[i]
        angles[index] = angle
        index+=1
        angle+=1
    for i in range(0, 301):
        if scan[i]<=LOOK_AHEAD_DISTANCE and scan[i]!=0:
            distances[index] = scan[i]
        angles[index] = angle
        index+=1
        angle+=1
    
    for i in distances:
        if distances[i]==0 and distances[i-1]!=0:
            curr_zero_gap_length+=1
            start_gap = i
        elif distances[i]==0:
            curr_zero_gap_length+=1
        else:
            if curr_zero_gap_length>max_zero_gap_length:
                max_zero_gap_length = curr_zero_gap_length
                max_start_gap = start_gap
            curr_zero_gap_length=0

    #in case gap at 400
    if curr_zero_gap_length > max_zero_gap_length:
        max_zero_gap_length = curr_zero_gap_length
        max_start_gap = start_gap
    
    if max_zero_gap_length!=0:
        start_angle = angles[max_start_gap]
        end_angle = angles[max_start_gap + max_zero_gap_length-1]
        middle_angle = (start_angle+end_angle)/2
    
    return middle_angle
        



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

    rc.drive.set_max_speed(0.34)
    speed = 1

    angle = gap_follow()
    if angle!=-10000:
        angle = angle/300
    print("angle", angle)

    rc.drive.set_speed_angle(speed, angle)

        




# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
