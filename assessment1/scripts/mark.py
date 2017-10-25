import rospy
import roslib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import math

class Markers:
	def __init__(self, topic = "/scanMarkers"):
		self.i = 0
		self.markers = []
		self.pub = rospy.Publisher(topic, MarkerArray, queue_size = 5000)
		Marker.LINE_STRIP

	def delete(self, index):
		self.markers.pop(index)
	
	def add_particles(self, particles):
		for p in particles:
			mr = Marker()
			mr.header.frame_id = 'map'
			mr.ns = "basic"
			mr.id = self.i
			mr.type = mr.SPHERE
			mr.action = mr.ADD
			mr.pose.position.x = p[0]
			mr.pose.position.y = p[1]
			mr.pose.orientation.w = 1
			mr.scale.x = 0.5
			mr.scale.y = 0.5
			mr.scale.z = 0.5
			mr.color.r = 1
			mr.color.g = 0
			mr.color.b = 0
			mr.color.a = 1.0
			self.markers.append(mr)
			self.i += 1
        
	def add_line(self, points, r, g, b, frame, size = 0.5):
		mr = Marker()
		mr.header.frame_id = frame
		mr.ns = "basic"
		mr.id = self.i
		mr.type = mr.LINE_STRIP
		mr.action = mr.ADD
		for p in points:
			mr.points.append(p)
		mr.scale.x = size
		mr.scale.y = size
		mr.scale.z = size
		mr.color.r = r
		mr.color.g = g
		mr.color.b = b
		mr.color.a = 1.0
		self.markers.append(mr)
		self.i += 1
			
	def add(self, x, y, r, g, b, frame, size = 0.5):
		mr = Marker()
		mr.header.frame_id = frame
		mr.ns = "basic"
		mr.id = self.i
		mr.type = mr.SPHERE
		mr.action = mr.ADD
		mr.pose.position.x = x
		mr.pose.position.y = y
		mr.pose.orientation.w = 1
		mr.scale.x = size
		mr.scale.y = size
		mr.scale.z = size
		mr.color.r = r
		mr.color.g = g
		mr.color.b = b
		mr.color.a = 1.0
		self.markers.append(mr)
		self.i += 1
        
	def add_robot(self, x, y, yaw, r, g, b, frame_id):
		self.add(x, y, r, g, b, frame_id)
		p = []
		p1 = Point(); p1.x = x; p1.y = y
		p2 = Point();
		p2.x = p1.x + 2 * math.cos(yaw); p2.y = p1.y + 2 * math.sin(yaw); #polar coord
		p = [p1, p2]
		self.add_line(p, r*3/4, g*3/4, b*3/4, frame_id, size = 0.25)
        
	def draw(self):
		markerArray = MarkerArray()
		for m in self.markers:
			markerArray.markers.append(m)
		self.pub.publish(markerArray)
		self.i = 0
	def clear(self):
		markerArray = MarkerArray()
		for m in self.markers:
			m.action = m.DELETE
			markerArray.markers.append(m)
		self.pub.publish(markerArray)
            
m = Markers()
m.add(2, 2, 2, 2, 2, 'base_link')
