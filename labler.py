#!/usr/bin/python

import os
import rospy
import sys
import json
import numpy as np
import cv2
from rs_yolo.msg import DetectionArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

File = None
counter = None
image = None
Output = None

class labeler():
    def __init__(self):
        if counter == 0:
            with open('data.json','w') as outfile:
                json.dump('[', outfile)

        self.sub = rospy.Subscriber('/vision', DetectionArray, self.callback)
        self.pub = rospy.Publisher('/camera/left/undistorted', Image, queue_size=1)

        # Needs from image conversion from CV2 to Ros message Image.
        bridge = CvBridge()

        if counter == len(File):
            rospy.loginfo("Program Complete, shutting down.")
            with open('data.json', 'w') as outfile:
                json.dump(']', outfile)
            exit(0)

# Process the Image with labels
    def callback(self, detectionArray):
        image_width , image_height = cv.GetSize(image)

        # using Mat image = load image from file name
        for i in detectionArray:

            detection = detectionArray.detections[i]

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

        json.dump({'annotations': [output], 'class': 'image',
            'filename': File[counter]}, outfile, sort_keys=True, indent=4, )

        output = None

        if counter != len(File):
            counter += 1
            Image = cv2.imread(sys.argv[1] + '/' + File[counter])
            image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.pub.publish(image_message)

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
    bridge = CvBridge()
    image = cv2.imread(sys.argv[1] + '/' + File[0])
    #bitmap = cv.CreateImageHeader((image.shape[1], image.shape[0]), cv.IPL_DEPTH_8U, 3)
#cv.SetData(bitmap, image.tostring(), image.dtype.itemsize * 3 * image.shape[1])
    rospy.loginfo("Listening to /vision for detection array and publishing to /camera/left/undistorted as Mat image")
    image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    pub = rospy.Publisher('/camera/left/undistorted', Image, queue_size=1)
    pub.publish(image_message)
    labeler = labeler()
    rospy.spin()
