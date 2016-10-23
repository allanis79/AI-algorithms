#!/usr/bin/env python

'''
http://www.redblobgames.com/pathfinding/a-star/implementation.html
'''
import collections

from implementation import * 

import heapq

class SimpleGraph:
	def __init__(self):
		self.edges={}

	def neighbors(self, id):
		return self.edges[id]

example_graph = SimpleGraph()

example_graph.edges= {
					'A':['B'],
					'B':['A','C','D'],
					'C':['A'],
					'D':['E','A'],
					'E':['B']
}

#print example_graph.neighbors('B')

class Queue:

	def __init__(self):
		self.elements = collections.deque()

	def empty(self):
		return len(self.elements) == 0 

	def put(self,x):
		self.elements.append(x)

	def get(self):
		return self.elements.popleft()


def bredth_first_search_1(graph,start):
	frointer = Queue()
	frointer.put(start)
	visited={}
	visited[start] = True 

	while not frointer.empty():
		current = frointer.get()
		print "Visiting %r" % current
		for next in graph.neighbors(current):
			print "neighbors %r" % next
			if next not in visited:
				print 'not visited %r' %next
				frointer.put(next)
				visited[next] = True 


bredth_first_search_1(example_graph,'A')

class SquareGrid: 
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.walls = [] 

	def in_bounds(self,id):
		(x,y) = id 
		return 0<=x < self.width and 0<=y < self.height

	def passable(self,id):
		return id not in self.walls

	def neighbors(self,id):
		(x,y) = id 
		results=[(x+1,y),(x,y-1),(x-1,y),(x,y+1)]
		if (x+y) % 2 == 0:
			results.reverse()

		results = filter(self.in_bounds,results)
		results = filter(self.passable, results)
		return results

g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS
draw_grid(g)


def bredth_first_search_2(graph,start):
	frointer = Queue()
	frointer.put(start)
	came_from={}
	came_from['start']= None 

	while not frointer.empty():
		current = frointer.get()

		for next in graph.neighbors(current):
			if next not in came_from:
				frointer.put(next)
				came_from[next] = current 
	return came_from

g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS

parents = bredth_first_search_2(g,(8,7))
draw_grid(g,width=2,point_to=parents,start=(8,7))


def bredth_first_search_3(graph,start,goal):
	frointer = Queue()
	frointer.put(start)
	came_from={}
	came_from['start'] = None 

	

	while not frointer.empty(): 
		current = frointer.get()

		if current == goal:
			break 

		for next in graph.neighbors(current):
			if next not in came_from:
				frointer.put(next)
				came_from[next] = current 

	return came_from


g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS

parents = bredth_first_search_3(g,(8,7),(20,12))
draw_grid(g,width=2,point_to=parents,start=(8,7),goal =(20,12))

class GridWithWeights(SquareGrid):
	def __init__(self,width,height):
		SquareGrid.__init__(self,width,height)
		self.weights = {} 

	def cost(self,from_node,to_node):
		return self.weights.get(to_node,1)


class PriorityQueue:
	def __init__(self):
		self.elements=[]

	def empty(self):
		return len(self.elements) == 0 

	def put(self, item, priority):
		heapq.heappush(self.elements,(priority,item))

	def get(self):
		return heapq.heappop(self.elements)[1]

def dijkstra_search(graph,start,goal):
	frointer = PriorityQueue()
	frointer.put(start,0)
	came_from={}
	cost_so_far={}
	came_from[start] = None 
	cost_so_far[start] = 0 

	while not frointer.empty():
		current = frointer.get()

		if current == goal:
			break 

		for next in graph.neighbors(current):
			new_cost = cost_so_far[current] + graph.cost(current,next)
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost 
				priority = new_cost 
				frointer.put(next,priority)
				came_from[next] =  current

	return came_from, cost_so_far


def reconstruct_path(came_from,start,goal):
	current = goal 
	path =[current]

	while current != start: 
		current = came_from[current]
		path.append(current)

	path.append(start)
	path.reverse()
	return path 

came_from, cost_so_far = dijkstra_search(diagram4, (1,4),(7,8))
draw_grid(diagram4,width=3,point_to=came_from,start=(1,4),goal=(7,8))
print 
draw_grid(diagram4, width=3, number=cost_so_far, start=(1, 4), goal=(7, 8))
print
draw_grid(diagram4, width=3, path=reconstruct_path(came_from, start=(1, 4), goal=(7, 8)))

def heurisitc(a,b): 
	(x1,y1) = a
	(x2,y2) = b 
	return  abs(x1-x2) + abs(y1-y2)

def a_star_search(graph,start,goal): 
	frointer = PriorityQueue()
	frointer.put(start,0)
	came_from={}
	cost_so_far={}
	came_from[start] = None 
	cost_so_far[start] = 0


	while not frointer.empty(): 
		current = frointer.get()

		if current == goal:
			break 

		for next in graph.neighbors(current):
			new_cost = cost_so_far[current] +graph.cost(current,next)
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost 
				priority = new_cost + heurisitc(goal, next)
				frointer.put(next,priority)
				came_from[next] = current 

	return came_from, cost_so_far 



came_from, cost_so_far = a_star_search(diagram4, (1, 4), (7, 8))
#print came_from, cost_so_far
draw_grid(diagram4, width=3, point_to=came_from, start=(1, 4), goal=(7, 8))
print
draw_grid(diagram4, width=3, number=cost_so_far, start=(1, 4), goal=(7, 8))




