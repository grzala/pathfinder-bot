#!/usr/bin/env python
import roslib
import rospy
import tf
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

from mark import Markers

import math, copy

class Tracker:

	def __init__(self):
		rospy.init_node('Tracker')
		self.subscriber = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.true_pos)
		self.sub_markers_dropped = rospy.Subscriber('/drop_mark', Point, self.drop_mark)
		self.markers_dropped = []
		
		self.br = tf.TransformBroadcaster()
		self.listen = tf.TransformListener()
		
		self.robot_drawer = Markers('/visualise/real_robot_pose')
		self.path_drawer = Markers('/visualise/true_path')
		
		self.true_pos = {
			'pose': Point(0, 0, 0),
			'orient': None,
			'euler': 0
		}
		
		self.points_visited = []
		self.last_time = rospy.get_time()
		
		while not rospy.is_shutdown():
			time_delta = rospy.get_time() - self.last_time
			if time_delta > 2: #every n seconds
				self.last_time = rospy.get_time()
				self.points_visited.append(copy.deepcopy(self.true_pos['pose']))
				
			self.draw()
			rospy.sleep(0.2)
		
	def true_pos(self, data):
		pose = data.pose.pose
		coord = pose.position
		orient = pose.orientation
		
		#publish transform
		self.br.sendTransform((coord.x, coord.y, 0), (0, 0, orient.z, orient.w), rospy.Time.now(), "/real_robot_pose", "map")#broadcast transform
		
		orient = tf.transformations.euler_from_quaternion((0, 0, orient.z, orient.w))[2]
		
		self.true_pos['pose'] = data.pose.pose.position
		self.true_pos['orient'] = data.pose.pose.orientation
		self.true_pos['euler'] = orient
		
	def draw(self):
		#drawing
		if len(self.markers_dropped) > 0:
			for m in self.markers_dropped:
				self.path_drawer.add(m.x, m.y, 1, 0, 0, 'map', 1)
		if len(self.points_visited) >= 2:
			self.path_drawer.add_line(self.points_visited, 1, 0, 1, 'map', size = 0.75)
		self.path_drawer.draw()
		self.robot_drawer.add_robot(self.true_pos['pose'].x, self.true_pos['pose'].y, self.true_pos['euler'], 1, 1, 1, 'map')
		self.robot_drawer.draw()
	
	def drop_mark(self, data):
		pos = self.true_pos['pose']
		self.markers_dropped.append(Point(pos.x, pos.y, 0))

if __name__ == '__main__':
	t = Tracker()
	rospy.spin()
