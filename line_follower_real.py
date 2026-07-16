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

kP = -0.001125   # maybe need to increase current kp hopefully will work
MAX_SPEED = 1  
kD = 0#-0.000001425
MIN_CONTOUR_AREA = 30
                    
# >> Constants
# The smallest contour we will recognize as a valid contour


# A crop window for the floor directly in front of the car
CROP_FLOOR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width()))
BLUE = ((91, 13, 109), (111, 255, 255))  # The HSV range for the color blue USE THIS FOR TRACK

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
error = 0
last_error = 0





########################################################################################
# Functions
########################################################################################


# [FUNCTION] Finds contours in the current color image and uses them to update
# contour_center and contour_area
def update_contour():
   global contour_center
   global contour_area


   image = rc.camera.get_color_image()


   if image is None:
       contour_center = None
       contour_area = 0
   else:
       # Crop the image to the floor directly in front of the car
       image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])


       # TODO Part 2: Search for line colors, and update the global variables
       # contour_center and contour_area with the largest contour found. shorten w/ function


       contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
       contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)


       if contour is not None:
           contour_center = rc_utils.get_contour_center(contour)
           contour_area = rc_utils.get_contour_area(contour)
           rc_utils.draw_contour(image, contour)
           rc_utils.draw_circle(image, contour_center)
       else:
           contour_center = None
           contour_area = 0
           
       # Display the image to the screen
       #rc.display.show_color_image(image)






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

   #rc.telemetry.declare_variables("angle", "contour_center")


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
   
   rc.drive.set_max_speed(MAX_SPEED)
   #speed = 1

   # Search for contours in the current color image
   update_contour()

   
   if contour_center is not None:
       setpoint = 360
       present_value = contour_center[1]
       error = setpoint - present_value
       angle = kP * error + kD * (error - last_error)/rc.get_delta_time()

       angle = rc_utils.clamp(angle, -1, 1)
  
   # Use the triggers to control the car's speed
   #rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
   #lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
   last_error = error
   speed = 1

   #rc.telemetry.record(angle, contour_center)
   #rc.telemetry.visualize()
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
   print("speed", speed, "angle", angle, "contour_center", contour_center, "contour_area", contour_area, "error", error)
   if rc.camera.get_color_image() is None:
       # If no image is found, print all X's and don't display an image
       print("X" * 10 + " (No image) " + "X" * 10)
   else:
       # If an image is found but no contour is found, print all dashes
       if contour_center is None:
           print("-" * 32 + " : area = " + str(contour_area))


       # Otherwise, print a line of dashes with a | indicating the contour x-position
       else:
           s = ["-"] * 32
           s[int(contour_center[1] / 20)] = "|"
           print("".join(s) + " : area = " + str(contour_area))




########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################


if __name__ == "__main__":
   rc.set_start_update(start, update, update_slow)
   rc.go()


