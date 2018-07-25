import os
import numpy as np
import cv2

from skimage.morphology import thin, skeletonize
from skimage import img_as_ubyte

import json

# http://imagecolorpicker.com/

# HTML code:	#854A12
# RGB code:	R: 133 G: 74 B: 18
# HSV:	29.22 86.47% 52.16%

# HTML code:	#EE800F
# RGB code:	R: 238 G: 128 B: 15
# HSV:	30.4 93.7% 93.33%


# Reads a input file path and uses a color filter to create a binary mask
def create_mask(file_path):
	orange = np.uint8([[[15, 128, 238]]])
	# convert a BGR value of yellow to HSV
	hsv_orange = cv2.cvtColor(orange, cv2.COLOR_BGR2HSV)

	# Have a threshold of +/- 10 for yellow threshold
	lower_yellow = hsv_orange - np.array([10, 200, 200])
	upper_yellow = hsv_orange + np.array([10, 0, 0])
	# print(hsv_yellow, lower_yellow, upper_yellow)


	# Load an color image
	img = cv2.imread(file_path)
	res = cv2.resize(img,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
	# Convert BGR to HSV
	hsv = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	# Bitwise-AND mask and original image
	res1 = cv2.bitwise_and(res,res, mask= mask)

	# Convert a 0,255 image to 0,1 binary image
	mask[mask>200] = 1
	# mask1 = skeletonize(mask)
	# mask1[mask1==1] = 255
	# mask1 = img_as_ubyte(mask1)
	return res, mask

# Takes all the box results, dumps the output as a json in the necessary file
def generate_json(box_results, filename):
	box_results_json = []
	for key, box_array in box_results.iteritems():
		box_json = {}
		box_json["class"] = "image"
		box_json["filename"] = key
		box_json["unlabeled"] = False
		box_json["annotations"] = []
		for box in box_array:
			box_value = {}
			(x,y,x1,y1,class_) = box
			box_value["class"] = class_
			box_value["type"] = "rect"
			box_value["x"] = x
			box_value["y"] = y
			box_value["height"] = y1-y
			box_value["width"] = x1-x

			box_json["annotations"].append(box_value)
		box_results_json.append(box_json)

	print(box_results_json)
	with open(filename, 'w') as f:
		json.dump(box_results_json, f, \
			sort_keys=True, indent=4, separators=(',', ': '))
	return box_results_json

def using_contour(mask):
	output = cv2.findContours(np.copy(mask), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	im2, contours, hierarchy = output
	print("output", len(contours))
	for cnt in contours:
		area = cv2.contourArea(cnt)
		new_mask = np.zeros_like(mask)
		if(area > 750):
			print("cnt area ", area)
			# cv2.drawContours(image=new_mask, 
			# 	contours=[cnt], contourIdx=0, color=(255), thickness=-1)
			# new_mask[new_mask>200] = 1
			x,y,w,h = cv2.boundingRect(cnt)
			print(x,y,w,h)
			return 2*x,2*y,2*w,2*h


# Get the list of png images from directory
def get_img_list(base_path):
	input_list = []
	for file in os.listdir(base_path):
	    if file.endswith(".png"):
	        # print(os.path.join(BASE_PATH, file))
	        input_list.append(file)
	return input_list


# Dump the undistroted images for all the images
def run_for_all_images(base_input):
	filename_list = get_img_list(base_input)
	box_list = {}
	for filename in filename_list[:]:
		print(filename)
		filepath_in = os.path.join(base_input, filename)
		# filepath_out = os.path.join(base_output, filename)
		res, mask = create_mask(filepath_in)
		
		# undistort(filepath_in, filepath_out)
		cv2.imwrite("resize.jpg", res)
		cv2.imwrite("mask.jpg", mask)
		box_list[filename] = using_contour(mask)
	return box_list

# Takes all the box results, dumps the output as a json in the necessary file
def generate_json(box_results, filename):
	box_results_json = []
	for key, box in box_results.iteritems():
		box_json = {}
		box_json["class"] = "image"
		box_json["filename"] = key
		box_json["unlabeled"] = False
		box_json["annotations"] = []

		box_value = {}
		(x,y,w,h) = box
		box_value["class"] = "pathmarker"
		box_value["type"] = "rect"
		box_value["x"] = x
		box_value["y"] = y
		box_value["height"] = h
		box_value["width"] = w

		box_json["annotations"].append(box_value)
		box_results_json.append(box_json)

	print(box_results_json)
	with open(filename, 'w') as f:
		json.dump(box_results_json, f, \
			sort_keys=True, indent=4, separators=(',', ': '))
	return box_results_json

# Using the box results information, reads all the input images and dumps
# the boxed images. The images are resized and dumps as JPEG so that the file size is reduced
def dump_output_images(box_results, input_base, output_base):

	for filename, box in box_results.iteritems():
		input_file = os.path.join(input_base, filename)
		output_file = os.path.join(output_base, filename[:-4]+".jpg")
		# Load an color image in grayscale
		img = cv2.imread(input_file)
		res = cv2.resize(img,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
		clr_ar = [(0,0,255),(0,255,0),(255,0,0)]

		(x,y,w,h) = box
		x /=2 ; y /=2 ; w /= 2; h /= 2
		cv2.rectangle(res,(x,y), (x+w,y+h), clr_ar[0] , 2)
		cv2.imwrite(output_file, res)
		# cv2.imshow(file_path,255*new_mask)
		# cv2.imshow(file_path, img)


	return

if __name__ == '__main__':
	# BASE_PATH_INPUT = "../../../../data/robosub/path_marker_sub_001_UNDIST"
	# BASE_PATH_OUTPUT = "../../../../data/robosub/path_marker_sub_001_boxed"
	BASE_PATH_INPUT = "../../../../data/robosub/path_marker_sub_001_UNDIST"
	BASE_PATH_OUTPUT = "../../../../data/robosub/path_marker_sub_001_boxed"

	box_list = run_for_all_images(BASE_PATH_INPUT)
	generate_json(box_list, os.path.join(BASE_PATH_OUTPUT, "output.json"))
	dump_output_images(box_list, BASE_PATH_INPUT, BASE_PATH_OUTPUT)