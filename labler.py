#!/usr/bin/python

import os
import rospy
import sys
import json
import numpy as np
import cv2
from std_msgs.msg import String
from rs_yolo.msg import DetectionArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

File = None
counter = None
image = None
Output = None

class labeler():
    def __init__(self):

        self.sub = rospy.Subscriber('/vision', DetectionArray, self.callback)
        self.pub = rospy.Publisher('/camera/left/undistorted', Image, queue_size=1)

        # Needs for image conversion from CV2 to Ros message Image.
        bridge = CvBridge()

        if counter == 0:
            print(sys.argv[1] + '/' + File[counter])
            image = cv2.imread(sys.argv[1] + '/' + File[counter])
            print(image)
            print("Hello")
            self.image_message = bridge.cv2_to_imgmsg(image, "bgr8")
            print(File[counter])
            print(self.image_message)
            self.pub.publish(self.image_message)

            with open('data.json','w') as outfile:
                json.dump('[', outfile)



# Process the Image with labels
    def callback(self, detectionArray):

        image_width = self.image_message.width
        image_height = self.image_message.height

        # using Mat image = load image from file name
        for i in detectionArray.detections: #TODO fix for loop to use integer, now uses i as detection.detections[i]

            detection = i

            x = detection.x * image_width
            y = detection.y * image_height

            width = detection.width * image_width / 2
            height = detection.height * image_height / 2

            # get box dimensions
            x = (x - detection.width)
            if x < 0:
                left = 0

            y = (y - detection.height)
            if y < 0:
                y = 0

            output.insert(i, {'class': detection.label, 'height': height,
                'type':'rect', 'width': width, 'x': x, 'y': y})
        with open('data.json','w') as outfile:
            json.dump({'annotations': [output], 'class': 'image',
                'filename': File[counter]}, outfile, sort_keys=True, indent=4, )

        output = None

        if counter != len(File):
            counter += 1
            image = cv2.imread(sys.argv[1] + '/' + File[counter])
            self.image_message = bridge.cv2_to_imgmsg(image, "bgr8")
            self.pub.publish(self.image_message)

        if counter == len(File):
            rospy.loginfo("Program Complete, shutting down.")
            with open('data.json', 'w') as outfile:
                json.dump(']', outfile)
            exit(0)

def printUsage():
    print("Invalid arguments")
    print("Usage:")
    print("\t./labeler.py (insert path to the folder here)")

if __name__ == "__main__":
    rospy.init_node('labeler')
#If path was not provided as an argument exit.
    if len(sys.argv) == 1:
        printUsage()
        exit(1)
    File = os.listdir(sys.argv[1])
    counter = 0

    rospy.loginfo("Listening to /vision for detection array and publishing to /camera/left/undistorted as Mat image")

    labeler = labeler()
    rospy.spin()
