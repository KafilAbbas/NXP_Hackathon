# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import math

from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan
import numpy as np
QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25
single_vector = 0

VECTOR_IMAGE_HEIGHT_PERCENTAGE = 0.40 
dist = 0
speed = SPEED_MAX

def speed_change(change,speed,speed_max):
		if change == "acc":
			if speed + 0.001 <= speed_max:
				speed = speed + 0.001
		elif change == "dcc":
			if speed - 0.02 >= speed_max:
				speed = speed - 0.0
		return speed

def calc_middle_x(point1, point2, distance,direction):
    # Calculate the midpoint of the line segment
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    # Calculate the slope of the original line segment
    if x2 - x1 != 0:
        slope = (y2 - y1) / (x2 - x1)
        # Calculate the slope of the perpendicular bisector
        perp_slope = -1 / slope
    else:
        # Special case: the original line segment is vertical, so the perpendicular bisector is horizontal
        perp_slope = 0

    # Calculate the angle of the perpendicular bisector
    angle = math.atan(perp_slope)

    # Calculate the coordinates of the new point
    if direction == "L":
        new_x1 = mx + distance * math.cos(angle)
        new_y1 = my + distance * math.sin(angle)
        return np.array([new_x1, new_y1])
    else:
        new_x2 = mx - distance * math.cos(angle)
        new_y2 = my - distance * math.sin(angle)
        return np.array([new_x2, new_y2])

    






class LineFollower(Node):
	""" Initializes line follower node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('line_follower')

		# Subscription for edge vectors.
		self.subscription_vectors = self.create_subscription(
			EdgeVectors,
			'/edge_vectors',
			self.edge_vectors_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for joy (for moving the rover in manual mode).
		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Subscription for traffic status.
		self.subscription_traffic = self.create_subscription(
			TrafficStatus,
			'/traffic_status',
			self.traffic_status_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for LIDAR data.
		self.subscription_lidar = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			QOS_PROFILE_DEFAULT)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.ramp_detected = False

	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left turn for positive, right turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, turn]

		self.publisher_joy.publish(msg)

	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""
	
			
		
		
	def edge_vectors_callback(self, message):
		global single_vector,dist,speed
		
		turn = TURN_MIN

		vectors = message
		# print(vectors.distance_list)
		# distance = message
		# print("this is distance ",distance)
		half_width = vectors.image_width / 2
		lower_image_height = int(vectors.image_height * VECTOR_IMAGE_HEIGHT_PERCENTAGE)
		rover_point = [vectors.image_width / 2, lower_image_height]
		# NOTE: participants may improve algorithm for line follower.

		if (vectors.vector_count == 0):  # none.
			speed = SPEED_25_PERCENT
			single_vector = 0
			pass

		if (vectors.vector_count == 1):  # curve.
			# speed = SPEED_50_PERCENT
			# Calculate the magnitude of the x-component of the vector.
			check_direction = vectors.vector_1[1].x - vectors.vector_1[0].x
			single_vector = single_vector + 1
			direction = ""
			if check_direction > 0:
				direction = "R"
			else:
				direction = "L"
			# print("this is full vector ",vectors.vector_1)
			# bottom_point = np.array([vectors.vector_1[1].x,vectors.vector_1[1].y])
			# middle_point = np.array([(vectors.vector_1[1].x + vectors.vector_1[0].x)/2,(vectors.vector_1[1].y + vectors.vector_1[0].y)/2])
			# print("this is with one vector ")
			# middle_x = 
			# distance = np.linalg.norm(bottom_point - rover_point)
			# print("this is length ",length_1)
			# print("this is new distance ",distance)
			# deviation = dist - distance
			# turn = deviation / half_width
			middle_x  = calc_middle_x(vectors.vector_1[1],vectors.vector_1[0],half_width + 120,direction)
			# middle_x = 160 + distance
			# deviation = 0
			# if single_vector > 10:
			speed = speed_change("acc",speed,SPEED_75_PERCENT)
			deviation = half_width - middle_x[0]
			turn = deviation / half_width
				# if length_1 != 0:
				# 	turn = 100/length_1
			# print("this is deviation from bottom point ",deviation)
			# deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
			# print("this is vector_1 when only one vector  ",vectors.vector_1)
			# print("this is deviation ",deviation)
			# if abs(length_1) < 200:
				
				# turn = length_1 / vectors.image_width
# 
			# print("this is turn ",turn)
			# 

		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors.

			
			length_1 = np.linalg.norm (np.array([vectors.vector_1[0].x,vectors.vector_1[0].y]) - np.array([vectors.vector_1[1].x,vectors.vector_1[1].y]))
			length_2 = np.linalg.norm (np.array([vectors.vector_2[0].x,vectors.vector_2[0].y]) - np.array([vectors.vector_2[1].x,vectors.vector_2[1].y]))
			bottom_point_1 = np.array([vectors.vector_1[1].x,vectors.vector_1[1].y])
			bottom_point_2 = np.array([vectors.vector_2[1].x,vectors.vector_2[1].y])
			middle_point_1 = np.array([(vectors.vector_1[1].x + vectors.vector_1[0].x)/2,(vectors.vector_1[1].y + vectors.vector_1[0].y)/2])
			middle_point_2 = np.array([(vectors.vector_2[1].x + vectors.vector_2[0].x)/2,(vectors.vector_2[1].y + vectors.vector_2[0].y)/2])
			# print("this is length ",length_1,length_2)
			# print()
			
			distance_1 = np.linalg.norm(bottom_point_1 - rover_point)
			distance_2= np.linalg.norm(bottom_point_2 - rover_point)
			# print("This is the distance from both vectors ",distance_1, distance_2)
			# print(distance_1,distance_2)
			# print("this is with two vectors ")
			single_vector = 0
			speed = speed_change("acc",speed,SPEED_MAX)
			if abs(length_1 - length_2) > 80:
				# speed = SPEED_50_PERCENT
				# if length_1 > length_2:
				# 	# turn = -0.1
				# 	turn  - 
				# else:
				# 	turn = 0.1
				# if single_vector > 2:
				print("i am in ", length_1 - length_2)
				if length_1 > length_2:
					check_direction = vectors.vector_1[1].x - vectors.vector_1[0].x
					if check_direction > 0:
						direction = "R"
					else:
						direction = "L"
					middle_x  = calc_middle_x(vectors.vector_1[1],vectors.vector_1[0],half_width + 120,direction)
					# turn = (length_2 - length_1)/(half_width*50)
					deviation = half_width - middle_x[0]
					turn = deviation / half_width
				else:
					check_direction = vectors.vector_2[1].x - vectors.vector_2[0].x
					if check_direction > 0:
						direction = "R"
					else:
						direction = "L"
					middle_x  = calc_middle_x(vectors.vector_2[1],vectors.vector_2[0],half_width + 120 ,direction)
					# turn = (length_2 - length_1)/(half_width*50)
					deviation = half_width - middle_x[0]
					turn = deviation / half_width
				# print("this is turn with diffrence > 80 ",turn)
			else:
				# speed = SPEED_MAX
				middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
				middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
				middle_x = (middle_x_left + middle_x_right) / 2
				deviation = half_width - middle_x
				dist = middle_x - middle_x_right
				turn = deviation / half_width
				# print("this is turn without difference ",turn)
			# print("this is turn ",turn)

		if (self.traffic_status.stop_sign is True):
			speed = SPEED_MIN
			print("stop sign detected")

		if self.ramp_detected is True:
			# TODO: participants need to decide action on detection of ramp/bridge.
			# SPEED_50_PERCENT
			print("ramp/bridge detected")

		if self.obstacle_detected is True:
			# TODO: participants need to decide action on detection of obstacle.
			print("obstacle detected")

		self.rover_move_manual_mode(speed, turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message

	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.

		shield_vertical = 4
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)

		# Get the middle half of the ranges array returned by the LIDAR.
		length = float(len(message.ranges))
		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
		side_ranges_right = ranges[0: int(length * theta / PI)]
		side_ranges_left = ranges[int(length * (PI - theta) / PI):]

		# process front ranges.
		angle = theta - PI / 2
		for i in range(len(front_ranges)):
			if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
				self.obstacle_detected = True
				return

			angle += message.angle_increment

		# process side ranges.
		side_ranges_left.reverse()
		for side_ranges in [side_ranges_left, side_ranges_right]:
			angle = 0.0
			for i in range(len(side_ranges)):
				if (side_ranges[i] < THRESHOLD_OBSTACLE_HORIZONTAL):
					self.obstacle_detected = True
					return

				angle += message.angle_increment

		self.obstacle_detected = False


def main(args=None):
	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
