#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point as Geo_Point, PoseWithCovarianceStamped, Twist, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
import sys
import types
import math

from cell import Cell, Point, CellList
from pathfinder import Mapper, AStar, Salesman
from mark import Markers
		
import tf

def biased_equal(x, y):
    return abs(x - y) < 0.2

class Navigate:
	
	def __init__(self, speed = 1.5, cell_size = 3):
		#init node
		rospy.init_node("navigate")
		rospy.sleep(0.3) #sometimes time is not properly initialized when i ask for amcl pose
		
		#init values
		self.speed = speed
		self.cell_size = cell_size
		self.listen = tf.TransformListener()
		self.laser_err = 0.3
		self.rotating = False
		
		#initialize markers
		self.path_drawer = Markers('/visualise/calculated_path')
		self.robot_drawer = Markers('/visualise/belief_robot_pose')
		
		#divide map into cells
		print "dividing map into cells..."
		m = Mapper(self.cell_size)
		self.clist = m.clist
		print "map divided"
		
		#init_pose
		amcl_start = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=10)
		self.pose = [0, 0, 0]
		self.pose[0] = amcl_start.pose.pose.position.x; self.pose[1] = amcl_start.pose.pose.position.y
		orient = amcl_start.pose.pose.orientation
		self.pose[2] = tf.transformations.euler_from_quaternion((0, 0, orient.z, orient.w))[2]
		
		p1 = rospy.get_param("/p1", self.pose)
		p2 = rospy.get_param("/p2", self.pose)
		p3 = rospy.get_param("/p3", self.pose)
		p4 = rospy.get_param("/p4", self.pose)
		p5 = rospy.get_param("/p5", self.pose)
		p6 = rospy.get_param("/p6", self.pose)
		
		#initialize publishers and subscribers
		self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_pos)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom)
		self.scan = LaserScan()
		self.last_odom = [0, 0, 0]
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.pub_drop = rospy.Publisher('/drop_mark', Geo_Point, queue_size=10)
		
		self.points_to_visit = [
			Point(self.pose[0], self.pose[1]), #first point must be initial_pose
			Point(p1[0], p1[1]),
			Point(p2[0], p2[1]),
			Point(p3[0], p3[1]),
			Point(p4[0], p4[1]),
			Point(p5[0], p5[1]),
			Point(p6[0], p6[1]),
		]
		
		#check if any destination is in an occupied cell
		for p in self.points_to_visit:
			if self.clist.is_occupied(p.x, p.y):
				print "Point: (%f, %f) is in an occupied cell" % (p.x, p.y)
				print "I can't go there"
				print "exiting..."
				exit(0)
	
		print "calculating paths..."
		s = Salesman(self.points_to_visit, self.clist.get_filtered())
		paths = s.build_path()
		print "paths calculated"
		
		self.cruising = True
		
		#for drawing
		self.markers_dropped = []
		self.points_to_draw = []
		for path in paths:
			for p in path:
				self.points_to_draw.append(Geo_Point(p[0], p[1], 0))
		
		print "moving"
		while not rospy.is_shutdown():
			if self.cruising:
				destination = paths[0][0]
				self.go_to(destination)
				
				#when point reached, pop it from array and place marker
				if self.distance_sqr(self.pose, destination) < 0.2:
					paths[0].pop(0)
					if len(paths[0]) <= 0: 
						#self.markers_dropped.append(Point(self.pose[0], self.pose[1]))
						#message can be empty
						self.pub_drop.publish(Geo_Point(0, 0, 0))
						paths.pop(0)
					if len(path) <= 0: 
						print "visited all points"
						self.cruising = False
			
			#draw
			self.draw()
			
			#sleep
			rospy.sleep(0.2)
			
	def draw(self):
		if len(self.markers_dropped) > 0:
			for m in self.markers_dropped:
				self.robot_drawer.add(m.x, m.y, 1, 0, 0, 'map', 1)
		self.robot_drawer.add_robot(self.pose[0], self.pose[1], self.pose[2], 0, 1, 0, 'map')
		self.robot_drawer.draw()
		
		for point in self.points_to_visit:
			self.path_drawer.add(point.x, point.y, 0, 1, 0, 'map', 0.75)
		self.path_drawer.add_line(self.points_to_draw, 0, 0, 1, 'map')
		self.path_drawer.draw()
	
	def go_to(self, destination):
		tw = Twist()

		dest_rotation = math.atan2(destination[1] - self.pose[1], destination[0] - self.pose[0])
		yaw = self.pose[2]

		dif = dest_rotation - yaw
		if dif < -math.pi: dif += 2*math.pi 
		if dif > math.pi: dif -= 2*math.pi

		if abs(dif) > 0.1:
			self.rotating = True
			if dif > 0: 
				tw.angular.z = 0.5
			elif dif < 0: 
				tw.angular.z = -0.5
		else: #go
			self.rotating = False
			tw.linear.x = self.speed

		self.pub.publish(tw)
			
	def distance_sqr(self, a, b):
		return ((a[0] - b[0])**2) + ((a[1] - b[1])**2)
			
	
	def get_pos(self, data):
		if not self.rotating:
			self.pose[0] = data.pose.pose.position.x 
			self.pose[1] = data.pose.pose.position.y
			orient = data.pose.pose.orientation
			self.pose[2] = tf.transformations.euler_from_quaternion((0, 0, orient.z, orient.w))[2]
	
	def get_odom(self, data):
		orient = data.pose.pose.orientation
		data = data.pose.pose.position
		x_delta = data.x - self.last_odom[0]
		y_delta = data.y - self.last_odom[1]
		
		orient = tf.transformations.euler_from_quaternion((0, 0, orient.z, orient.w))[2]
		theta_delta = orient - self.last_odom[2]
		self.pose[0] += x_delta; self.pose[1] += y_delta; self.pose[2] += theta_delta;
		self.last_odom = [data.x, data.y, orient]
		
if __name__ == '__main__':
	speed = 1
	cell_size = 3
	n = Navigate(speed, cell_size)
