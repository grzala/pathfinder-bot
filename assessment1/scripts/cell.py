import heapq, types

class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.free = True
		
	def inside(self, cell):
		return self.x >= cell.x and self.x <= cell.x + cell.width and self.y >= cell.y and self.y <= cell.y + cell.height
		
	def get_cost(self, point):
		return abs(point.x - self.x) + abs(point.y - self.y)
	
	def is_equal(self, point):
		return self.x == point.x and self.y == point.y
		
class Cell:
	def __init__(self, x, y, width, height):
		self.x = x
		self.y = y
		self.height = height
		self.width = width
		self.center = Point(x + width/2, y + height/2)
		self.points = []
		self.neighbours = []
	
	#fitler out occupied neighbours
	def get_filtered_neighbours(self):
		n = []
		for nn in self.neighbours:
			if not nn.is_occupied(): n.append(nn)
		return n
		
	def is_equal(self, c):
		return self.x == c.x and self.y == c.y and self.width == c.width and self.height == c.height
		
	def length(self):
		return len(self.points)
	
	def is_occupied(self):
		return self.length() > 0
	
	def get_size(self):
		w = self.width
		h = self.height
		size = (w * h)
		return round(size)
		
	def __repr__(self):
		return self.__str__()
	
	def __str__(self):
		str = "[%d, %d, %d, %d]" % (self.x, self.y, self.width, self.height)
		return str 
		
	def get_cost(self, cell):
		return abs(cell.center.x - self.center.x) + abs(cell.center.y - self.center.y)
		
class CellList:
	
	cell_list = {}
	
	def build(self, oc, min_side):
		init_x = oc.info.origin.position.x
		init_y = oc.info.origin.position.y
		grid_width = oc.info.width; grid_height = oc.info.height;
		res = oc.info.resolution
		if res > min_side: min_side = res
		width = grid_width * res; height = grid_height * res;
		self.cell_width = min_side; self.cell_height = min_side
		
		#translate obstacles from grid to coordinates
		points = []
		x = 0; y = 0;
		for cell in oc.data:
			p = Point(init_x + x, init_y + y)
			if cell == 100: points.append(p)
			x += res
			if x >= width:
				x = 0 
				y += res
			if y >= height: break
		
		#arrange cells intoo a dictionary
		#so that operation take not O(n)
		#but O(r+c) where r is the number of rows and c is the number of columns
		cells = []
		x = 0
		y = 0
		while y < width:
			self.cell_list[init_y + y] = []
			while x < height:
				self.cell_list[init_y + y].append(Cell(init_x + x, init_y + y, min_side, min_side))
				x += min_side
			x = 0
			y += min_side
		
		#sort
		for key in self.cell_list:
			self.cell_list[key] = sorted(self.cell_list[key], self.x_compare)
		
		#arrange points
		for p in points:
			target_cell = None
			k = None
			for key in self.cell_list:
				if key <= p.y and key + self.cell_height >= p.y:
					k = key
					break
			for cell in self.cell_list[key]:
				if cell.x <= p.x and cell.x + self.cell_width >= p.x:
					target_cell = cell
					break
			target_cell.points.append(p)
			
		#build neighbour grid in O(n) time
		for key in self.cell_list:
			for i in range(len(self.cell_list[key])):
				mn = min(self.cell_list.keys(), key=int)
				mx = max(self.cell_list.keys(), key=int)
				#x
				if i > 0: self.cell_list[key][i].neighbours.append(self.cell_list[key][i-1])
				if i < len(self.cell_list[key])-1: self.cell_list[key][i].neighbours.append(self.cell_list[key][i+1])
				#y
				if key > mn: self.cell_list[key][i].neighbours.append(self.cell_list[key-min_side][i])
				if key < mx: self.cell_list[key][i].neighbours.append(self.cell_list[key+min_side][i])
				
				''' FOR DIAGONAL ASTAR - REQUIRES BIGGER CELLS
				if i > 0 and key > mn: self.cell_list[key][i].neighbours.append(self.cell_list[key-min_side][i-1])
				if i > 0 and key < mn: self.cell_list[key][i].neighbours.append(self.cell_list[key+min_side][i-1])
				if i < len(self.cell_list[key])-1 and key > mn: self.cell_list[key][i].neighbours.append(self.cell_list[key-min_side][i + 1])
				if i < len(self.cell_list[key])-1 and key < mx: self.cell_list[key][i].neighbours.append(self.cell_list[key+min_side][i + 1])
				'''
	
	def is_occupied(self, x, y):
		target_cell = None
		k = None
		for key in self.cell_list:
			if key <= y and key + self.cell_height >= y:
				k = key
				break
		for cell in self.cell_list[key]:
			if cell.x <= x and cell.x + self.cell_width >= x:
				target_cell = cell
				break
		
		if target_cell == None: return True #edge of map
		return target_cell.is_occupied()
		
	#for sorting
	def x_compare(self, cell1, cell2):
		if cell1.x < cell2.x:
			return -1
		elif cell1.x > cell2.x:
			return 1
		else:
			return 0
		
	#get unoccupied cells
	def get_filtered(self):
		cells = []
		for key in self.cell_list:
			for cell in self.cell_list[key]:
				if not cell.is_occupied(): cells.append(cell)
		return cells
	
	#get occupied cells
	def get_occupied(self):
		cells = []
		for key in self.cell_list:
			for cell in self.cell_list[key]:
				if cell.is_occupied(): cells.append(cell)
		return cells
		
	#get cells as list
	def get_cells(self):
		cells = []
		for key in self.cell_list:
			for cell in self.cell_list[key]:
				cells.append(cell)
		return cells
		

