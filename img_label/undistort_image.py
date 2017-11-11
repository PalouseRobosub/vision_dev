import cv2
import os
import numpy as np


# HTML code:	#854A12
# RGB code:	R: 133 G: 74 B: 18
# HSV:	29.22° 86.47% 52.16%

# HTML code:	#EE800F
# RGB code:	R: 238 G: 128 B: 15
# HSV:	30.4° 93.7% 93.33%


# Reference:
#
# stereo_out_camera_data.xml
# http://answers.opencv.org/question/174305/cv2fisheye-camera-calibration-python/
# https://github.com/PalouseRobosub/robosub/blob/bagUndistorter/src/vision/util/UnbagAndUndistort.cpp
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html


# Undistortion function which converts input image to a undistorted output
def undistort(filename_in, filename_out):
	image = cv2.imread(filename_in)
	# mask = create_mask(image)
	# image = mask*image
	(height, width, _) = image.shape
	K1 = [[3.0750757411242284e+02, 0., 6.8544508305028876e+02],
		  [0., 3.0543095843427369e+02, 5.2521504097457353e+02],
		  [0., 0., 1.]]
	K1 = np.asarray(K1)
	D1 = [2.4061840587975818e-02, -5.0302192696572649e-02,
		9.2085796367517903e-02, -5.4046288453628057e-02]
	D1 = np.asarray(D1)
	R1 = [
	[9.9927266119343083e-01, -4.7748856939934707e-03, -3.7833174040891794e-02],
	[4.4174306295589535e-03, 9.9994486779303071e-01, -9.5261577255150200e-03],
	[3.7876574528752538e-02, 9.3521035595020684e-03, 9.9923866181257226e-01]]
	R1 = np.asarray(R1)
	P1 = [[4.6480510867137906e+02, 0., 7.0819126384509525e+02, 0.],
		  [0., 4.6480510867137906e+02, 5.1306773257708687e+02, 0.],
		  [0., 0., 1., 0.]]
	P1 = np.asarray(P1)
	mapx,mapy = cv2.initUndistortRectifyMap(K1, D1, R1, \
		P1, (width, height), cv2.CV_16SC2)
	dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
	# cv2.imwrite("undistort.png", dst)
	# cv2.imwrite("original.png", image)
	cv2.imwrite(filename_out, dst)
	return 

# Get the list of png images from directory
def get_img_list(base_path):
	input_list = []
	for file in os.listdir(base_path):
	    if file.endswith(".png"):
	        # print(os.path.join(BASE_PATH, file))
	        input_list.append(file)
	return input_list

# Dump the undistroted images for all the images
def run_for_all_images(base_input, base_output):
	filename_list = get_img_list(base_input)
	for filename in filename_list:
		print(filename)
		filepath_in = os.path.join(base_input, filename)
		filepath_out = os.path.join(base_output, filename)
		undistort(filepath_in, filepath_out)

	return

# Not used
def create_mask(image):
	(height, width, _) = image.shape
	mask = np.zeros(image.shape)
	radius = 500
	circle_mask = cv2.circle(mask, (width/2, height/2), radius, \
		(1,1,1), -1)
	# print(circle_mask)
	cv2.imwrite("check.png", circle_mask)
	return circle_mask

if __name__ == '__main__':
	# BASE_PATH_INPUT = "../../../../data/robosub/path_marker_sub_000/"
	# BASE_PATH_OUTPUT = "../../../../data/robosub/path_marker_sub_000_UNDIST"
	BASE_PATH_INPUT = "../../../../data/robosub/path_marker_sub_001/"
	BASE_PATH_OUTPUT = "../../../../data/robosub/path_marker_sub_001_UNDIST"

	# file_list = U.get_img_list(BASE_PATH)
	# img = undistort(file_list[0])
	# create_mask(img)
	run_for_all_images(BASE_PATH_INPUT, BASE_PATH_OUTPUT)