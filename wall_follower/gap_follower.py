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
import sys


sys.path.insert(1, '../../library')
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
forward_distance = 0.0
kD = 0.00000
kP = 0.005  #fine???? idk
last_error = 0
dots = None

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
   global speed
   global angle
   global dots

   # Initialize variables
   speed = 0
   angle = 0


   # Set initial driving speed and angle
   rc.drive.set_speed_angle(speed, angle)


   # Set update_slow to refresh every half second
   rc.set_update_slow_time(0.5) # Remove 'pass' and write your source code for the start() function here
   dots = rc.display.new_matrix()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global last_error

    rc.drive.set_max_speed(.34)


    
    scan = rc.lidar.get_samples()
    present_value = 0
    max=150
    prev_max=0
    far=[]
    zero_indices=[]
    num=scan.__len__()
    low_scan_angle=int((-90/360)*num)
    high_scan_angle=int((90/360)*num)
    for i in range((low_scan_angle), (high_scan_angle)):
        if scan[i] == 0:
            far.append(scan[i])
            zero_indices.append(i)
            # if (scan[i-1]==0 and i>0) or (i<scan.__len__() and scan[i+1] == 0):
            #     far.append(scan[i])
            #     zero_indices.append(i)
    if far.__len__() == 0:
        for i in range(0, scan.__len__()):
            if rc_utils.get_lidar_average_distance(scan, i, 5) > max:
                max=rc_utils.get_lidar_average_distance(scan, i, 5)
                max_angle=i
                
    if far.__len__() > 0:
        setpoint=zero_indices[far.__len__()//2]
        farthest_distance=setpoint
    else:
        setpoint=max_angle
        farthest_distance=setpoint
    error=(setpoint-present_value)
    angle=kP*error
    angle=rc_utils.clamp(angle, -1, 1)
    speed=1
    last_error=error
    angle_correspond=[]
    used_dots=dots[0,0:23], dots[0:7, 0], dots[0:7, 23]
    for i in range(0,7):
         angle_correspond.append((dots[i,0],-90+(i*180/(37)), -90+180/37+(i*180/(37)))) #for first column   
    second_angle=angle_correspond[angle_correspond.__len__()-1][angle_correspond.__len__()-1][angle_correspond.__len__()-1]
    for i in range(0,23):
           angle_correspond.append((dots[0,i],second_angle+(i*180/(37)), second_angle+180/37+(i*180/(37)))) #for first row
    third_angle=angle_correspond[angle_correspond.__len__()-1][angle_correspond.__len__()-1][angle_correspond.__len__()-1]
    for i in range(0,7):
           angle_correspond.append((dots[i,23], third_angle+(i*180/(37)), second_angle+180/37+(i*180/(37)))) #for last column

    min=float('inf')
    diff=float('inf')
    good_dot=None
    for i in range(0, angle_correspond.__len__()):
        diff=abs(angle_correspond[i][1]-setpoint)
        if diff < min:
            min=diff
            good_dot=angle_correspond[i][0]
    for i in range(0, good_dot.__len__()):
        good_dot[i]=1
        rc.display.set_matrix(dots)

    print ("speed", speed, "angle", angle, "setpoint", setpoint, "farthest distance", max, "forward distance", scan[0], "error", error, "present_value", present_value)


    #print("angle", angle, "setpoint", setpoint, "farthest distance", present_value, "forward distance", forward_distance)

    rc.drive.set_speed_angle(speed, angle)
    #print("angle", angle, "setpoint", setpoint, "farthest distance", max, "forward distance", scan[0])
        




# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    #print("angle", angle, "left forward", left_forward_distance, "right forward", right_forward_distance)
    pass

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
