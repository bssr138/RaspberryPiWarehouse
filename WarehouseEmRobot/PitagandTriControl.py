#!usr/bin/env python3

#---------------------------------------------------------------------------------------------
# Detecting geomtry and tracking Aruco tags using a Raspberry Pi Camera and mobile robot
# with and without [GPIO] serial (UART) communication
#---------------------------------------------------------------------------------------------

# camera modules
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
# opencv modules
import cv2 as cv
import numpy
import numpy as np
import sys
# serial communication
import serial
import time
from gpiozero import LED

# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# ser.flushInput()
centerpin = 24 #pin 13
leftpin = 22 #pin15
rightpin = 23 #pin16

sys.path.append('/usr/local/lib/python3.9/dist-packages')
# GPIO modules & initialization
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(27,GPIO.OUT) # turn right Flag
GPIO.setup(17,GPIO.OUT) # Defective Flag to send to Propeller
GPIO.setup(18,GPIO.OUT) # Non-defective Flag to send to Propeller
GPIO.setup(centerpin,GPIO.OUT)
GPIO.setup(leftpin,GPIO.OUT)
GPIO.setup(rightpin,GPIO.OUT)
GPIO.setup(3,GPIO.OUT) #Enemy Flag
GPIO.setup(2,GPIO.OUT) #Friendly Flag

#7-segment LED stuff
# Define the GPIO pins for each segment (a, b, c, d, e, f, g)
segments = (26, 19, 13, 6, 5, 21, 20)

# Create LED objects for each segment
leds = [LED(segment) for segment in segments]

# Define numbers 0-9 and their corresponding segments
# A number is represented as a tuple with the segments that should be turned on
numbers = {
    '0': (0, 1, 2, 3, 4, 5),
    '1': (1, 2),
    '2': (0, 1, 6, 4, 3),
    '3': (0, 1, 6, 2, 3),
    '4': (5, 6, 1, 2),
    '5': (0, 5, 6, 2, 3),
    '6': (0, 5, 4, 3, 2, 6),
    '7': (0, 1, 2),
    '8': (0, 1, 2, 3, 4, 5, 6),
    '9': (0, 1, 2, 5, 6),
}

def display_number(number):
    # Turn off all segments
    for led in leds:
        led.off()

    # Turn on the segments for the given number
    for segment in numbers[number]:
        leds[segment].on()

# load camera
PiCamera._set_exposure_mode = 10

# function to check if tag ID is even
def is_even(tag_id):
    return tag_id % 2 == 0

#=============================================================================================
# getTagNumbers(image,drawMarker = False):
# This Function to takes an image as input and outputs the tag ids of the aruco tags detected 
# and the pixel coordinates of the corners of the aruco tag in the iamge.
#=============================================================================================
def getTagNumbers(image,drawMarker = False):
    #Convert image to grayscale
    grayScaleIm = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    RIm = cv.rotate(grayScaleIm,cv.ROTATE_180)
    #cv.waitKey(0)
    #Get the arcuo tag dictionary for the type of tag we want
    arucoTagDict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    #Create the aruco parameter var
    tagParams = cv.aruco.DetectorParameters_create()
    #Detect the tags
    (corners, tagIds, rejected) = cv.aruco.detectMarkers(RIm, arucoTagDict,parameters=tagParams)
    if (len(corners) > 0) and (drawMarker):
        cv.aruco.drawDetectedMarkers(RIm,corners)

    return corners,tagIds


def is_triangle(approx):
    return len(approx) == 3

def get_triangle_direction(triangle_points):
    p1, p2, p3 = triangle_points[:, 0, :]
    sorted_points = sorted([p1, p2, p3], key=lambda x: x[1])

    top, middle, bottom = sorted_points

    if middle[0] > top[0]:
        return "right"
    else:
        return "left"

#---------------------------------------------------------------------------------------------
# MAIN
#---------------------------------------------------------------------------------------------

#Setting up camera
# Initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.rotation = 0  # Adjust this if your camera is mounted differently
camera.exposure_mode = 'auto'
camera.exposure_compensation = 2  # Increase the value to brighten the image
rawCapture = PiRGBArray(camera)

#Sleep so camera has time to setup
sleep(0.1)

IdArray = [None] * (12) # array to store the tag IDs of detected tags
IdCounter = 0
detected_ids_set = set()

min_area = 1000
max_area = 20000
font = cv.FONT_HERSHEY_COMPLEX


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # serial communication
#     ser_bytes = ser.readline()
#     decoded_bytes = ser_bytes.decode('utf-8').rstrip()
# 
#     if decoded_bytes:
#         print(decoded_bytes)

    #Saving the captured frame
    myImage = frame.array      
    
    #Passing the frame through the getTagNumbers function
    (corners,detectedIds) = getTagNumbers(myImage)
    
    print(detectedIds)
    
    #! Handle the case when two tags are detected
    if detectedIds is not None and len(detectedIds) == 2:
        detected_ids_list = detectedIds.flatten().tolist()
        first_id_is_even = is_even(detected_ids_list[0])
        second_id_is_even = is_even(detected_ids_list[1])

        if first_id_is_even and second_id_is_even:
                #print("both non-defective")
                GPIO.output(18,GPIO.HIGH)
                GPIO.output(17,GPIO.LOW)
                sleep(0.5)
                GPIO.output(18,GPIO.LOW)
                sleep(0.5)
                GPIO.output(18,GPIO.HIGH)
                sleep(0.5)
                GPIO.output(18,GPIO.LOW)
            
        elif (first_id_is_even and not second_id_is_even) or (not first_id_is_even and second_id_is_even):
                #print("one defective, one non-defective")
                GPIO.output(18,GPIO.HIGH)
                GPIO.output(17,GPIO.HIGH)
                
        else:
                #print("both defective")
                GPIO.output(17,GPIO.HIGH)
                GPIO.output(18,GPIO.LOW)
                sleep(0.5)
                GPIO.output(17,GPIO.LOW)
                sleep(0.5)
                GPIO.output(17,GPIO.HIGH)
                sleep(0.5)
                GPIO.output(18,GPIO.LOW)

            
         # Check if any of the detected IDs is already present in the set
        if not any(tag_id in detected_ids_set for tag_id in detected_ids_list):
        # Add the newly detected tag IDs to the set
            detected_ids_set.update(detected_ids_list)

    #! Handle the case when only one tag is detected
    elif (detectedIds is not None and len(detectedIds) == 1):
        detected_ids_list = detectedIds.flatten().tolist()
        only_id_is_even = is_even(detected_ids_list[0])

        #! Tag following logic
        corner1_x = corners[0][0][0][0]
        corner2_x = corners[0][0][1][0]
        corner3_x = corners[0][0][2][0]
        corner4_x = corners[0][0][3][0]

        
        min_corner = min(corner1_x, corner2_x, corner3_x, corner4_x)
        print(min_corner)
        
        if min_corner<200:
            print("right")
            GPIO.output(leftpin, GPIO.LOW)
            GPIO.output(centerpin, GPIO.LOW)
            GPIO.output(rightpin, GPIO.HIGH)

        elif min_corner>=200 and min_corner<400:
            print("center")
            GPIO.output(leftpin, GPIO.LOW)
            GPIO.output(centerpin, GPIO.HIGH)
            GPIO.output(rightpin, GPIO.LOW)
        else:
            print("left")
            GPIO.output(leftpin, GPIO.HIGH)
            GPIO.output(centerpin, GPIO.LOW)
            GPIO.output(rightpin, GPIO.LOW)
        
        if only_id_is_even:
            GPIO.output(18,GPIO.HIGH)
            GPIO.output(17,GPIO.LOW)
                        
        else:
            GPIO.output(17,GPIO.HIGH)
            GPIO.output(18,GPIO.LOW)
            
         # Check if any of the detected IDs is already present in the set
        if not any(tag_id in detected_ids_set for tag_id in detected_ids_list):
        # Add the newly detected tag IDs to the set
            detected_ids_set.update(detected_ids_list)

    #! Handle the case when no tag is detected and but a blue triangle is detected
    else:
        # Converting image to HSV color space
        hsv = cv.cvtColor(myImage, cv.COLOR_BGR2HSV)

        # Defining the range of blue color in HSV
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Thresholding the HSV image to obtain only blue colors
        mask = cv.inRange(hsv, lower_blue, upper_blue)

        # Detecting contours in the image
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Going through every contour found in the image
        for cnt in contours:
            approx = cv.approxPolyDP(cnt, 0.09 * cv.arcLength(cnt, True), True)

            if is_triangle(approx):
                area = cv.contourArea(cnt)
                #print("area:", area)
                if area > min_area and area < max_area:
                    # Draws the boundary of the triangle
                    cv.drawContours(myImage, [approx], 0, (0, 255, 0), 5)

                    # Determine the direction of the triangle
                    direction = get_triangle_direction(approx)
                    print(f"Triangle points to the {direction}.")
                    
                    if (direction == "right"):
                        GPIO.output(27,GPIO.HIGH)
                        sleep(0.3)
                        
                    else:
                        GPIO.output(27,GPIO.LOW)

                    # Used to flatten the array containing the coordinates of the vertices
                    n = approx.ravel()

                    # Extract and display the triangle's corners
                    for i in range(0, len(n), 2):
                        x = n[i]
                        y = n[i + 1]

                        # String containing the co-ordinates
                        string = str(x) + " " + str(y)

                        # Text on the co-ordinates
                        cv.putText(myImage, string, (x, y), font, 0.8, (0, 0, 255))


        GPIO.output(18,GPIO.LOW)
        GPIO.output(17,GPIO.LOW)
    

    # Show the current frames
    cv.imshow("Frame", myImage)
    print(detected_ids_set)
    odd_count = sum(1 for tag_id in detected_ids_set if tag_id % 2 != 0)
    if(odd_count == 6):
        display_number(str(odd_count))
 

    # Clear the stream for the next frame
    rawCapture.truncate(0)

    # Break the loop if 'q' is pressed on the keyboard
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

#Cleaning up the capture   
cv.destroyAllWindows()  