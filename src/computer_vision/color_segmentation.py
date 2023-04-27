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

# Image Parameters
HEIGHT = 376
WIDTH = 672

# Occlusion Mask Parameters
PEAK_HEIGHT = int(float(HEIGHT) * 0.35) # What camera height do we want to full obscure the view?
VERTICAL_OFFSET = 650 # Decreasing raises the height where we start obscuring view edges (triangular mask)

# Line Following Parameters
LEFT_BOUND = int(WIDTH*(4.0/10.0)) # Decreasing this ignores potential left lanes further right
RIGHT_BOUND = int(WIDTH * (6.0/10.0))  # Increasing this ignores potential right lanes further left
VERTICAL_SLOPE = 0.25 # What slope of line do we want to consider a lane?

# Important Tuning Parameters
LOOKAHEAD = int(float(HEIGHT) * 0.75) # Decreasing this makes the target point further away from the camera

mask_dimensions = np.array([
				[(-VERTICAL_OFFSET, HEIGHT), (int(WIDTH * 0.5), PEAK_HEIGHT), (WIDTH+VERTICAL_OFFSET, HEIGHT)]
				])
lane_follower_mask = np.zeros((HEIGHT,WIDTH))
lane_follower_mask = cv2.fillPoly(lane_follower_mask, mask_dimensions, 255)

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


def cd_color_segmentation(image, obstruct_view=True, visualize=False):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	if isinstance(image, type(None)):
		return ((0,0),(0,0))
	
	# Convert image to HSL
	image_hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

	# Create Color Mask
	light_gray = (0, 170, 0)
	dark_gray = (255, 250, 255)
	color_mask = cv2.inRange(image_hls, light_gray, dark_gray)

	# Filter to Lane Color
	_,_, filtered_image = cv2.split(cv2.bitwise_and(image, image, mask=color_mask))
	if visualize:
		image_print(image)
		image_print(filtered_image)
	
	# Obstruct View to Lane
	if obstruct_view:
		lane_follower_mask = np.zeros_like(filtered_image)
		lane_follower_mask = cv2.fillPoly(lane_follower_mask, mask_dimensions, 255)
		filtered_image = cv2.bitwise_and(filtered_image, filtered_image, mask=lane_follower_mask)
	if visualize:
		image_print(filtered_image)

	# Find lines through Hough Transforms
	lines = cv2.HoughLinesP(filtered_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
	lines = lines if lines != None else []

	# Extract potential left and right lanes
	left_lanes = []
	right_lanes = []
	for line in lines:
		x_1, y_1, x_2, y_2 = line[0]
		slope, intercept = np.polyfit((x_1, x_2), (y_1, y_2), 1)
		if slope < -VERTICAL_SLOPE and x_1 < LEFT_BOUND and x_2 < RIGHT_BOUND:
			left_lanes.append((slope, intercept))
		elif slope > VERTICAL_SLOPE and x_1 > RIGHT_BOUND and x_2 > RIGHT_BOUND:
			right_lanes.append((slope, intercept))

	# Handle Missing Lane Recovery
	if len(left_lanes) == 0:
		left_lanes.append((-1.0,0))
	if len(right_lanes) == 0:
		right_lanes.append((0.5, -100))

	# Fit average left and right lane estimate
	left_slope, left_intercept = np.average(left_lanes, axis=0)
	right_slope, right_intercept = np.average(right_lanes, axis=0)

	# Calculate Pursuit Point
	left_target_x = (LOOKAHEAD-left_intercept) / left_slope
	right_target_x = (LOOKAHEAD-right_intercept) / right_slope
	target_point = (int((left_target_x+right_target_x)/2.0), LOOKAHEAD)
	
	""" Unsure if we need to constraint to image bounds
	target_point[0] = min(WIDTH,max(0,target_point[0]))
	target_point[1] = min(HEIGHT,max(0,target_point[1]))
	target_point = tuple(target_point)
	"""

	# Visualize Pursuit Point
	if visualize:
		cv2.line(filtered_image, (0, int(left_intercept)), (WIDTH, int(float(WIDTH)*left_slope+left_intercept)), (255,255,255),3)
		cv2.line(filtered_image, (0, int(right_intercept)), (WIDTH, int(float(WIDTH)*right_slope+right_intercept)), (255,255,255),3)
		cv2.circle(filtered_image, target_point, radius=10, color=(255,255,255), thickness=-1)
		image_print(filtered_image)
		cv2.circle(image, target_point, radius=10, color=(255,255,255), thickness=-1)
		image_print(image)
	
	return target_point


if __name__ == "__main__":
	cd_color_segmentation(cv2.imread("./racetrack_images/lane_3/image" + str(43) + ".png"), obstruct_view=True, visualize=True)
	"""
	for i in range(1,70):
		print(i)
		cd_color_segmentation(cv2.imread("./racetrack_images/lane_3/image" + str(i) + ".png"), obstruct_view=True, visualize=True)
	"""