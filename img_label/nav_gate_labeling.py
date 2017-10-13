
import os
import numpy as np
import cv2

from skimage.morphology import thin, skeletonize
from skimage import img_as_ubyte

import json

# Reads a input file path and uses a color filter to create a binary mask
def create_mask(file_path):
	yellow = np.uint8([[[0, 255, 255]]])
	# convert a BGR value of yellow to HSV
	hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)

	# Have a threshold of +/- 10 for yellow threshold
	lower_yellow = hsv_yellow - np.array([10, 200, 200])
	upper_yellow = hsv_yellow + np.array([10, 0, 0])
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


# Try getting a rectangle box for boundary for a horizontal bar given the mask
def get_rectangle(mask):

	hor_sum = np.sum(mask, axis=1)
	ver_sum = np.sum(mask, axis=0)
	# print(mask.shape, hor_sum.shape, ver_sum.shape)
	# print(hor_sum)
	# print(ver_sum)


	# Each segment requires a lower threshold for identifying the longer dimension
	# width for horizontal bar and height for vertical bar
	# Find lower bar width first
	ver_sum[(ver_sum<75) & (ver_sum > 2)]
	ver_lt = 2*int(np.average(ver_sum[(ver_sum<75) & (ver_sum > 0)]))
	ver_ht = ver_lt
	print("vert th ", ver_ht)#, ver_sum)

	non_zero_ver_ht = np.where(ver_sum > ver_ht)
	horrod_left = 0
	horrod_right = len(ver_sum)
	# print(non_zero_ver_ht)

	# if(non_zero_ver_ht[0].size):
	# 	horrod_left = max(np.min(non_zero_ver_ht), horrod_left)
	# 	horrod_right = min(np.max(non_zero_ver_ht), horrod_right)

	non_zero_ver_lt = np.where((ver_sum[horrod_left:horrod_right] > 0) & \
		(ver_sum[horrod_left:horrod_right] < ver_lt))
	if(non_zero_ver_lt[0].size):
		horrod_left = max(np.min(non_zero_ver_lt), horrod_left)
		horrod_right = min(np.max(non_zero_ver_lt), horrod_right)
	
	# print(\
	# 	#np.max(non_zero_ver_ht), np.min(non_zero_ver_ht), \
	# 	np.min(non_zero_ver_lt), np.max(non_zero_ver_lt), \
	# 	horrod_left, horrod_right)


	hor_lt = 2*int(np.average(hor_sum[(hor_sum<75) & (hor_sum > 0)]))+5
	hor_ht = hor_lt
	non_zero_ver_ht = np.where(hor_sum > hor_ht)
	print("hort th ", hor_ht)#, hor_sum)
	horrod_top = 0
	horrod_bot = len(hor_sum)
	if(non_zero_ver_ht[0].size):
		horrod_top = max(np.min(non_zero_ver_ht), horrod_top)-int(ver_lt/4)
		horrod_bot = min(np.max(non_zero_ver_ht), horrod_bot)+int(ver_lt/4)
		return 2*horrod_left, 2*horrod_top, 2*horrod_right, 2*horrod_bot

	else:
		return 0, 0, 0, 0

# Using the get rectangle function, the below function calls it thrice to get the three bars
# It checks if there is a horizontal rod. If yes then it splits the images into two vertical images
# Passes a 90 degree rotated (transposed) image to identify the vertical poles as two horizontal poles
def get_three_rectangles(mask):
	box_array = []
	x,y,x1,y1 = get_rectangle(mask)
	if(x!=0 or y!=0 or x1!=0 or y1!=0):
		box_array.append((x,y,x1,y1,"nav_channel_bar"))
	else:
		print(box_array, x, y, x1, y1)
		return box_array
	# cv2.rectangle(res,(x,y), (x1,y1), (255,0,0), 2)

	horrod_left = x; horrod_right = x1
	# use it to split the image into two
	ver_mid_val = int((horrod_left + horrod_right)/4)
	print("Mid value ", ver_mid_val)

	# then using the separate images try identifying the vertical poles

	# ver_sum_left = ver_sum[:ver_mid_val]
	# hor_sum_left = np.sum(mask[:,:ver_mid_val], axis=1)
	x,y,x1,y1 = get_rectangle(mask[:,:ver_mid_val].T)
	if(x!=0 or y!=0 or x1!=0 or y1!=0):
		box_array.append((y,x,y1,x1,"nav_channel_post"))
	# cv2.rectangle(res,(y,x), (y1,x1), (0,255,0), 2)


	# ver_sum_right = ver_sum[ver_mid_val:]
	hor_sum_right = np.sum(mask[:,ver_mid_val:], axis=1)
	x,y,x1,y1 = get_rectangle(mask[:,ver_mid_val:].T)
	y+=2*ver_mid_val; y1+=2*ver_mid_val
	if(x!=0 or y!=0 or x1!=0 or y1!=0):
		box_array.append((y,x,y1,x1,"nav_channel_post"))
	# cv2.rectangle(res,(y,x), (y1,x1), (0,0,255), 2)

	print(box_array)
	return box_array

# Takes a mask, finds all the different contours. For each contour create a mask.
# Pass the mask to check whether they have the Navigation channel gate.
# It uses a basic threshold to identify the biggest of contour and ignore smaller blobs
# Also, if the detect rectangle returns a empty array, it says it is not a valid channel
# It keeps going until either all contours are validated or it finds one navigation channel gate
def using_contour(mask):
	output = cv2.findContours(np.copy(mask), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	im2, contours, hierarchy = output
	print("output", len(contours))
	for cnt in contours:
		area = cv2.contourArea(cnt)
		new_mask = np.zeros_like(mask)
		print("cnt area ", area)
		if(area > 750):
			cv2.drawContours(image=new_mask, 
				contours=[cnt], contourIdx=0, color=(255), thickness=-1)
			new_mask[new_mask>200] = 1

			box_array = get_three_rectangles(new_mask)
			if(len(box_array)):
				return new_mask, box_array
			# Pass a transpose of the image for detecting horizontal bars
			box_array = get_three_rectangles(new_mask.T)
			transposed_box_array = []
			if(len(box_array)):
				for box in box_array:
					(x,y,x1,y1,class_) = box
					transposed_box_array.append((y,x,y1,x1,class_))
				return new_mask, transposed_box_array


			# cv2.imshow('new_mask',255*new_mask)
	return mask, []

# Takes a list of all the input files and runs the segmentation for each file
def run_for_all_inputs(input_list):
	box_results = {}
	for file_path in input_list:
		file_name = file_path.split("/")[-1]
		print(file_name)

		img, mask = create_mask(file_path)
		new_mask, box_array = using_contour(mask)
		
		box_results[file_name] = box_array
	return box_results

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

# Using the box results information, reads all the input images and dumps
# the boxed images. The images are resized and dumps as JPEG so that the file size is reduced
def dump_output_images(box_results, input_base, output_base):

	for filename, box_array in box_results.iteritems():
		input_file = os.path.join(input_base, filename)
		output_file = os.path.join(output_base, filename[:-4]+".jpg")
		# Load an color image in grayscale
		img = cv2.imread(input_file)
		res = cv2.resize(img,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
		i = 0
		clr_ar = [(0,0,255),(0,255,0),(255,0,0)]
		for box in box_array:
			(x,y,x1,y1,_) = box
			x /=2 ; y /=2 ; x1 /= 2; y1 /= 2
			cv2.rectangle(res,(x,y), (x1,y1), clr_ar[i] , 2)
			i += 1
		cv2.imwrite(output_file, res)
		# cv2.imshow(file_path,255*new_mask)
		# cv2.imshow(file_path, img)


	return

# Edit the below variables:
# BASE_PATH: Path of the input folder containing all the input images
# BASE_PATH_OP: Path of the output folder to dump the JSON and the boxed image files
# The images are downsampled and rectangles are detected on it and then scaled by 2

if __name__ == '__main__':
	BASE_PATH = "../../../data/robosub/channel_sub_000/"
	BASE_PATH = "../../../data/robosub/channel_sub_001/"
	# BASE_PATH = "../../../data/robosub/channel_sub_000_error/"
	# BASE_PATH = "../../../data/robosub/channel_sub_001_error/"

	BASE_PATH_OP = "../../../data/robosub/channel_sub_000_OP/"
	BASE_PATH_OP = "../../../data/robosub/channel_sub_001_OP/"
	# BASE_PATH_OP = "../../../data/robosub/channel_sub_000_error_OP/"
	# BASE_PATH_OP = "../../../data/robosub/channel_sub_001_error_OP/"

	#left_318.575000.png"
	#left_296.175000.png"
	#left_322.575000.png"
	#left_322.375000.png" 
	#left_294.975000.png"
	# input_list = [
	# "../../../data/robosub/channel_sub_001/left_322.575000.png",
	# "../../../data/robosub/channel_sub_001/left_296.175000.png",
	# "../../../data/robosub/channel_sub_001/left_318.575000.png",
	# "../../../data/robosub/channel_sub_001/left_322.375000.png",
	# "../../../data/robosub/channel_sub_001/left_294.975000.png"
	# ]
	input_list = []
	for file in os.listdir(BASE_PATH):
	    if file.endswith(".png"):
	        # print(os.path.join(BASE_PATH, file))
	        input_list.append(os.path.join(BASE_PATH, file))
	print(len(input_list))
	box_results = run_for_all_inputs(input_list)
	# print(box_results)
	generate_json(box_results, os.path.join(BASE_PATH_OP, "output.json"))
	dump_output_images(box_results,BASE_PATH, BASE_PATH_OP)
	#run_for_all_inputs(input_list[200:210])
	#run_for_all_inputs(input_list[100:110])

	# cv2.imshow('frame',res)
	#cv2.imshow('mask',mask)
	#cv2.imshow('mask1',mask1)
	# while(1):
	# 	k = cv2.waitKey(5) & 0xFF
	# 	if k == 27:
	# 	    break
	# cv2.destroyAllWindows()

#####################################################################################
# Ignore all the functions below. They were used to evaluate different algorithms
#####################################################################################

def using_ver_hist():
	non_zero_hor = np.where(hor_sum > 30)
	print(len(non_zero_hor))
	if(non_zero_hor[0].size):
		horrod_top = np.min(non_zero_hor)-5
		horrod_bot = np.max(non_zero_hor)+15

		non_zero_ver = np.where((ver_sum > 0) & (ver_sum < 30))
		avg_thickness = int(np.average(ver_sum[non_zero_ver])+0.9)
		horrod_left = np.min(non_zero_ver)-15
		horrod_right = np.max(non_zero_ver)
		print(horrod_left,horrod_top, horrod_right,horrod_bot, avg_thickness)
		cv2.rectangle(res,(horrod_left,horrod_top),\
			(horrod_right,horrod_bot),(255,0,0),2)

	# Vertical rods
	non_zero_hor_low = np.where(hor_sum > 0)
	if(non_zero_hor[0].size):
		verrod_top = np.min(non_zero_hor_low)-10
		verrod_bot = np.max(horrod_top)+10

		mid_pt = int((horrod_left + horrod_right)/2)
		non_zero_ver_left_mid = np.where(ver_sum[:mid_pt] > 100)
		non_zero_ver_right_mid = np.where(ver_sum[mid_pt:] > 100)

		if(non_zero_ver_left_mid[0].size):
			leftverrod_left = np.min(non_zero_ver_left_mid)-15
			leftverrod_right = np.max(non_zero_ver_left_mid)+5
			cv2.rectangle(res,(leftverrod_left,verrod_top),\
				(leftverrod_right,verrod_bot),(0,255,0),2)

		if(non_zero_ver_right_mid[0].size):
			rightverrod_left = np.min(non_zero_ver_right_mid)+mid_pt-5
			rightverrod_right = np.max(non_zero_ver_right_mid)+mid_pt+15
			cv2.rectangle(res,(rightverrod_left,verrod_top),\
				(rightverrod_right,verrod_bot),(0,255,0),2)
	return

#####################################################################################
# Ignore all the functions below. They were used to evaluate different algorithms
#####################################################################################

def the_contour_idea():
	output = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	im2, contours, hierarchy = output
	print("output", len(output), len(contours))

	output1 = cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	im21, contours1, hierarchy1 = output1
	print("output", len(output1), len(contours1))

	i = 0
	for cnt in contours:
		area = cv2.contourArea(cnt)
		print("area", i , area, len(cnt))
		print(cnt)
		cv2.drawContours(res, [cnt], 0, (0,255,0), 3)
		if(area > 1500):
			x,y,w,h = cv2.boundingRect(cnt)
			cv2.rectangle(res,(x,y),(x+w,y+h),(255,0,0),2)
			
			epsilon = 0.001*cv2.arcLength(cnt,True)
			approx = cv2.approxPolyDP(cnt,epsilon,True)
			# cv2.drawContours(res, contours, i, (0,255,0), 3)
			cv2.drawContours(res, [approx], 0, (0,255,0), 3)
			area = cv2.contourArea(cnt)
			perimeter = cv2.arcLength(cnt,True)
			thickness = area/perimeter

			print(len(cnt), len(approx), approx, area, perimeter, thickness)
			print(np.where(mask1==255))
			x_list, y_list = np.where(mask1==255)

		i += 1

	print(img.shape, hsv.shape)



