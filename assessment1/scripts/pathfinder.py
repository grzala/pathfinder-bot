#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point as Geo_Point

import mark
import sys, math, heapq

from cell import Cell, Point, CellList
		
class Mapper:
	def __init__(self, min_side = 3.5):
		self.oc = OccupancyGrid()
		self.oc = rospy.wait_for_message('/map', OccupancyGrid, timeout=5)
		self.clist = self.fixed_decompose(min_side)
	
	def fixed_decompose(self, min_side):
		clist = CellList()
		clist.build(self.oc, min_side)
		return clist
			
#draw cells for debugging purposes
def add_bounds_to_draw(mr, cells):
	for cell in cells:
		p1 = Geo_Point(); p2 = Geo_Point(); p3 = Geo_Point(); p4 = Geo_Point();
		p1.x = cell.x; p1.y = cell.y;
		p2.x = cell.x + cell.width; p2.y = cell.y;
		p3.x = cell.x + cell.width; p3.y = cell.y + cell.height;
		p4.x = cell.x; p4.y = cell.y + cell.height;
		p = [p1, p2, p3, p4, p1]
		mr.add_line(p, 1, 0, 0, 'map')
			

class AStar:
	@staticmethod
	def perform_search(cells, start_point, goal_point):
		#find cells closest to points
		start_cell = None
		goal_cell = None
		min_dist = [sys.maxint, sys.maxint]
		for cell in cells:
			p = Point(cell.center.x, cell.center.y)
			dist = p.get_cost(start_point)
			if dist < min_dist[0]:
				min_dist[0]= dist
				start_cell = cell
			dist = p.get_cost(goal_point)
			if dist < min_dist[1]:
				min_dist[1]= dist
				goal_cell = cell

		head = AStar.PriorityQueue()
		head.push(start_cell, 0)
		came_from = {}; came_from[start_cell] = None
		cost_so_far = {}; cost_so_far[start_cell] = 0
		
		while not head.empty():
			current_cell = head.pop()

			if current_cell.is_equal(goal_cell): #break if we stumble upon goal
				break
			
			for next_cell in current_cell.get_filtered_neighbours(): #look through neighbours of the cell that is currently head of priority queue
				new_cost = cost_so_far[current_cell] + current_cell.get_cost(next_cell)
				
				#if it is closer to get to next_cell from current cell, overwrite cost, came_from and priority queue
				if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
					cost_so_far[next_cell] = new_cost
					priority = new_cost + goal_cell.get_cost(next_cell)
					head.push(next_cell, priority)
					came_from[next_cell] = current_cell

		points = []
		c = came_from[goal_cell]
		cc = []
		while not c.is_equal(start_cell): #backtrack to start
			cc.append(c)
			c = came_from[c]
		cc = cc[::-1] #flip to get path
		cc.insert(0, start_cell)
		cc.append(goal_cell)
		
		#convert to a list of points
		#get rid of multiple points in the same line
		points.append([start_point.x, start_point.y])
		points.append([cc[0].center.x, cc[0].center.y])
		for i in range(1, len(cc)-1):
			c0 = cc[i-1]
			c1 = cc[i]
			c2 = cc[i+1]
			if math.atan2(c1.y - c0.y, c1.x - c0.x) != math.atan2(c2.y - c1.y, c2.x - c1.x): #if points are not in the same line
				#points.append([c0.center.x, c0.center.y])
				points.append([c1.center.x, c1.center.y])
				#points.append([c2.center.x, c2.center.y])
		points.append([cc[-1].center.x, cc[-1].center.y])
		points.append([goal_point.x, goal_point.y])
		
		#ensure to remove duplicates
		points2 = []
		for p in points: 
			if p not in points2: points2.append(p)
		points = points2
		
		#total cost of the path
		cost = 0
		for i in range(1, len(points)):
			a = points[i-1]
			b = points[i]
			cost += math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
		
		return points, cost
		
	#use heapq to implement a priority queue data structure
	class PriorityQueue:
		def __init__(self):
			self.elements = []
		def empty(self):
			return len(self.elements) == 0
		def push(self, item, priority):
			heapq.heappush(self.elements, (priority, item))
		def pop(self):
			return heapq.heappop(self.elements)[1]

class Salesman:
	#travelling salesman problem
	
	#translate point list into weighted edges graph
	def __init__(self, points_to_visit, filtered_cells):
		self.edges = []
		for p in points_to_visit:
			e = self.Edge(p)
			for p2 in points_to_visit:
				if not p.is_equal(p2):
					(path, cost) = AStar.perform_search(filtered_cells, p, p2)
					e.add_path(p2, path, cost)
			self.add_edge(e)
		
	def add_edge(self, edge):
		self.edges.append(edge)
	
	def get_edge(self, point):
		for e in self.edges:
			if e.coord.is_equal(point):
				return e
	
	#closest neighbour heuristic
	def build_path(self):
		current_edge = self.edges[0]
		edges_visited = [current_edge]
		paths = []
		#closest neighbour heuristic
		while not len(edges_visited) == len(self.edges):
			target_p = None
			shortest_path_cost = sys.maxint
			shortest_path = []
			for target, attr in current_edge.paths.items():
				target = self.get_edge(target)
				if attr['cost'] < shortest_path_cost and target not in edges_visited:
					shortest_path_cost = attr['cost']
					shortest_path = attr['path']
					target_p = target
			current_edge = target_p
			edges_visited.append(current_edge)
			paths.append(shortest_path)
		
		return paths
	
	class Edge:
		def __init__(self, point):
			self.coord = point
			self.paths = {}
			
		def add_path(self, target, path, cost):
			self.paths[target] = {'path': path, 'cost': cost}
	

if __name__ == "__main__":
	rospy.init_node("teeest")
	mr = mark.Markers()
	m = Mapper(3)
	clist = m.clist
	cells = clist.get_cells()
	
	zero = [0, 0, 0]
	p1 = rospy.get_param("/p1", zero)
	p2 = rospy.get_param("/p2", zero)
	p3 = rospy.get_param("/p3", zero)
	p4 = rospy.get_param("/p4", zero)
	p5 = rospy.get_param("/p5", zero)
	p6 = rospy.get_param("/p6", zero)
	
	points_to_visit = [
			Point(p1[0], p2[1]),
			Point(p2[0], p2[1]),
			Point(p3[0], p3[1]),
			Point(p4[0], p4[1]),
			Point(p5[0], p5[1]),
			Point(p6[0], p6[1]),
		]
	
	for p in points_to_visit:
		mr.add(p.x, p.y, 1, 0, 0, 'map')
	
	s = Salesman(points_to_visit, clist.get_filtered())
	
	paths = s.build_path()
	points = []
	for path in paths:
		print path[0], path[-1]
		for p in path:
			gp = Geo_Point()
			gp.x = p[0]
			gp.y = p[1]
			points.append(gp)
	
	mr.add_line(points, 0, 1, 1, "map")

	print "drawing"
	add_bounds_to_draw(mr, clist.get_occupied())

	while not rospy.is_shutdown():
		mr.draw()
		
