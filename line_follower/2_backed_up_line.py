"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs


File Name: lab_f.py


Title: Lab F - Line Follower


Author: [PLACEHOLDER] << [Write your name or team name here]


Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines
of code under the #TODO indicators to complete the lab.


Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR moves forward at full speed
- When the left trigger is pressed, the RACECAR, moves backwards at full speed
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
"""


########################################################################################
# Imports
########################################################################################


import sys
import cv2 as cv
import numpy as np
import yaml


# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
#sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils


########################################################################################
# Global variables
########################################################################################


rc = racecar_core.create_racecar()
with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)

kP = config["kP"]
kD = config["kD"]
MIN_CONTOUR_AREA = config["MIN_CONTOUR_AREA"]
MAX_SPEED = config["MAX_SPEED"]

# >> Constants
# The smallest contour we will recognize as a valid contour


# A crop window for the floor directly in front of the car
CROP_FLOOR_NEAR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR_FAR = ((200, 0), (300, rc.camera.get_width()))
BLUE = ((93, 107, 122), (114, 255, 255))  # The HSV range for the color blue

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center_near = None  # The (pixel row, pixel column) of contour
contour_area_near = 0  # The area of contour
contour_center_far = None  # The (pixel row, pixel column) of contour
contour_area_far = 0  # The area of contour
error = 0
last_error = 0
timer = 0.0




########################################################################################
# Functions
########################################################################################


# [FUNCTION] Finds contours in the current color image and uses them to update
# contour_center and contour_area
def update_contour():
    global contour_center_near, contour_center_far
    global contour_area_near, contour_area_far

    image = rc.camera.get_color_image()

    if image is None:
        contour_center_near = contour_center_far = None
        contour_area_near = contour_area_far = 0
        return

    near_img = rc_utils.crop(image, CROP_FLOOR_NEAR[0], CROP_FLOOR_NEAR[1])
    contours_near = rc_utils.find_contours(near_img, BLUE[0], BLUE[1])
    contour_near = rc_utils.get_largest_contour(contours_near, MIN_CONTOUR_AREA)
    if contour_near is not None:
        contour_center_near = rc_utils.get_contour_center(contour_near)
        contour_area_near = rc_utils.get_contour_area(contour_near)
    else:
        contour_center_near = None
        contour_area_near = 0

    far_img = rc_utils.crop(image, CROP_FLOOR_FAR[0], CROP_FLOOR_FAR[1])
    contours_far = rc_utils.find_contours(far_img, BLUE[0], BLUE[1])
    contour_far = rc_utils.get_largest_contour(contours_far, MIN_CONTOUR_AREA)
    if contour_far is not None:
        contour_center_far = rc_utils.get_contour_center(contour_far)
        contour_area_far = rc_utils.get_contour_area(contour_far)
    else:
        contour_center_far = None
        contour_area_far = 0

    display_img = image.copy()

    if contour_center_near is not None:
        row_off, col_off = CROP_FLOOR_NEAR[0]
        full_center_near = (contour_center_near[0] + row_off, contour_center_near[1] + col_off)
        rc_utils.draw_circle(display_img, full_center_near, color=(0, 255, 0))  # green

    if contour_center_far is not None:
        row_off, col_off = CROP_FLOOR_FAR[0]
        full_center_far = (contour_center_far[0] + row_off, contour_center_far[1] + col_off)
        rc_utils.draw_circle(display_img, full_center_far, color=(0, 0, 255))  # red

    #rc.display.show_color_image(display_img)






# [FUNCTION] The start function is run once every time the start button is pressed
def start():
   global speed
   global angle
   global last_error


   # Initialize variables
   speed = 0
   angle = 0
   last_error = 0


   # Set initial driving speed and angle
   rc.drive.set_speed_angle(speed, angle)

   # Set update_slow to refresh every half second
   rc.set_update_slow_time(0.5)


   # Print start message
   print(
       ">> Lab 2A - Color Image Line Following\n"
       "\n"
       "Controls:\n"
       "   Right trigger = accelerate forward\n"
       "   Left trigger = accelerate backward\n"
       "   A button = print current speed and angle\n"
       "   B button = print contour center and area"
   )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed 
def update():
   """
   After start() is run, this function is run every frame until the back button
   is pressed
   """
   global speed
   global angle
   global error
   global last_error
   global timer
   
   rc.drive.set_max_speed(MAX_SPEED)

   # Search for contours in the current color image
   update_contour()

   timer = timer + rc.get_delta_time()

   if contour_center_far and contour_center_near is not None:
    setpoint = 360

    error_near = setpoint - contour_center_near[1] if contour_center_near else 0
    error_far  = setpoint - contour_center_far[1]  if contour_center_far  else error_near 

    W_NEAR, W_FAR = 0.7, 0.3
    error = W_NEAR * error_near + W_FAR * error_far

    angle = kP * error + kD * (error - last_error) / rc.get_delta_time()
    angle = rc_utils.clamp(angle, -1, 1)
  
   last_error = error
   speed = 1

   rc.drive.set_speed_angle(speed, angle)


   # Print the current speed and angle when the A button is held down


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a
# message every frame in update is computationally expensive and creates clutter
def update_slow():
   """
   After start() is run, this function is run at a constant rate that is slower
   than update().  By default, update_slow() is run once per second
   """

   # Print a line of ascii text denoting the contour area and x-position
   print("speed", speed, "angle", angle, "error", error, "timer", timer, "contour_center_near", contour_center_near)





########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################


if __name__ == "__main__":
   rc.set_start_update(start, update, update_slow)
   rc.go()
