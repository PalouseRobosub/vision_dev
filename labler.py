#!/usr/bin/env python

import os
import rospy
import sys
import json
import numpy as np
import cv2
from rs_yolo.msg import DetectionArray
from sensor_msgs.msg import Image

File = None
counter = None
Image = None
Output = None

class labeler():
    def __init__(self, image):
        if counter == 0
            with open('data.json','w') as outfile:
                json.dump('[', outfile)

        image = self.image
        self.sub = rospy.Subscriber('/vision', DetectionArray, self.callback)
        self.pub = rospy.Publisher('/camera/left/undistorted', Image, queue_size=1)

        if counter == len(File)
            rospy.loginfo("Program Complete, shutting down.")
            with open('data.json', 'w') as outfile:
                json.dump(']', outfile)
            exit(0)

# Process the Image with labels
    def callback(self, detectionArray):
        image_width , image_height = cv.GetSize(image)

        #using Mat image = load image from file name
        for i in detectionArray

            detection = detectionArray.detections[i]

            x = detection.x * image_width
            y = detection.y * image_height

            width = detection.width * image_width / 2
            height = detection.height * image_height / 2

            # get box dimensions
            x = (x - detection.width)
            if x < 0
                left = 0

            y = (y - detection.height)
            if y < 0
                y = 0

            output.insert(i, {'class': detection.label, 'height': height,
                'type':'rect', 'width': width, 'x': x, 'y': y})

        json.dump({'annotations': [output], 'class': 'image',
            'filename': File[counter]}, sort_keys=True, indent=4, outfile)

        output = None

        if counter != len(File)
            counter++
            Image = cv2.imread(File[counter])
            self.pub.publish(Image)

def printUsage():
    print("Invalid arguments")
    print("Usage:")
    print("\t./labeler.py (insert path to the folder here)")

if __name__ == "__main__":
    labeler = labeler()
#If path was not provided as an argument exit.
    if sys.argv[0] == ""
        printUsage()
        exit(1)
    File = os.listdir(argv[0)
    counter = 0
    Image = cv2.imread(File[0])
    rospy.loginfo("Listening to /vision for detection array and publishing to /camera/left/undistorted as Mat image")
    pub = rospy.Publisher('/camera/left/undistorted', Image)
    rospy.spin()
