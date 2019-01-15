#by Huaheng Yu (hy240), Maisy Chung (mlc340), Yi Lyu (yl866)
import numpy as np
import random

def MapGenerator(map_length):
	return np.zeros((map_length,map_length), dtype = bool)

def Manhatton_value(current, goal):
	return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def BlockedMapGenerator(map_length, Blocked_Percentage, start, goal):
	one_map = np.zeros((map_length,map_length), dtype = bool)
	Num_Blocked = int((map_length * map_length)*(Blocked_Percentage))
	j = 0 
	while j <= Num_Blocked:
		x = random.randint(0, map_length-1)
		y = random.randint(0, map_length-1)
		one_map[(x,y)] = True
		j += 1
	one_map[start] = False
	one_map[goal] = False
	return one_map

def vaild_neighbor(cell, length):
	p_list = [(cell[0],cell[1]+1),(cell[0],cell[1]-1),(cell[0]+1,cell[1]),(cell[0]-1,cell[1])]
	out = []
	for one_cell in p_list:
		if  -1< one_cell[0] < length and -1< one_cell[1] < length:
			out.append(one_cell)
	return out

def getPath(from_dict, current_cell , goal_cell):
	out = [goal_cell]
	next_cell = from_dict[goal_cell]
	while next_cell != current_cell:
		out.insert(0,next_cell)
		next_cell = from_dict[next_cell]
	out.insert(0,next_cell)
	return out

def getBackPath(from_dict, current_cell, goal_cell):
	out = [current_cell]
	next_cell = from_dict[current_cell]
	while next_cell != goal_cell:
		out.append(next_cell)
		next_cell = from_dict[next_cell]
	out.append(next_cell)
	return out
	
def UpdataBlocks( m_map, length, current_cell, blocklist):
	reachable_cells = vaild_neighbor(current_cell, length)
	for cell in reachable_cells:
		if blocklist[cell]:
			# 2  blocked
			m_map[2][cell] = 2
		else:
			# 1  empty
			m_map[2][cell] = 1
