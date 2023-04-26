import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

#Image
HEIGHT = 376
WIDTH = 672
top = np.zeros((int((4.5/8.0)*HEIGHT), WIDTH), dtype="uint8")
middle = np.ones((int((1.0/8.0)*HEIGHT), WIDTH), dtype="uint8")
bottom = np.zeros(((HEIGHT-top.shape[0]-middle.shape[0]), WIDTH), dtype="uint8")
line_follower_mask = np.vstack((top, middle, bottom))

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


def cd_color_segmentation(image, template=None, is_line_follower=False):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	if isinstance(image, type(None)):
		return ((0,0),(0,0))
	
	# Reduce image noise
	blurred_image = cv2.GaussianBlur(image, (3,3), 0)
	blurred_image = cv2.erode(blurred_image, (3,3))
	blurred_image = cv2.dilate(blurred_image, (3,3))

	# Convert image to HSL
	image_hls = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HLS)

	# Create Color Mask
	light_gray = (0, 190, 0)
	dark_gray = (255, 270, 255)
	color_mask = cv2.inRange(image_hls, light_gray, dark_gray)

	# Filter to Lane Color
	_,_, filtered_image = cv2.split(cv2.bitwise_and(image, image, mask=color_mask))
	image_print(filtered_image)

	# Obstruct image view
	if is_line_follower:
		filtered_image = cv2.bitwise_and(filtered_image, filtered_image, mask=line_follower_mask)

	# Extract lanes through Hough Transforms
	lanes = cv2.HoughLinesP(filtered_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)



cd_color_segmentation(cv2.imread("./racetrack_images/lane_3/image1.png"))
