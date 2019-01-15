#by Huaheng Yu (hy240), Maisy Chung (mlc340), Yi Lyu (yl866)

'''
version 1.0.5
	Implemented adaptive search (for some reason it is slower than forward with tie break)
	forward with my own tie break rule can have performence of around 0.5s for 101*101 maze
	Implemented user choice for different analyse between ways of search
	by M1
'''

'''
version 1.0.4
tie break implemented:
fixed backward 
user selection implemented
'''

'''
version 1.0.3
increased preformence 
	solve time for 101*101 improved from 60s to 3s
'''

'''
version 1.0.2
trying to implemnt tie brak along with heap
dum_heap created
'''

'''
version 1.0.1
didn't add any comment, lol 
if you want to change maze size, change l in main 
if you want to change start and goal, don't forget to change it in BlockedMapGenerator
	otherwise, your start and goal might be blocked 
code look's nesty, and i know it, will refine them later 
instruction: run in py3 and keep closing windows 
			Known Bug: color block in the first step is wrong, will  fix it later 
have fun XD
	by M1
'''
from ftr_utility import *
import numpy as np
from matplotlib.colors import ListedColormap
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import random
import dum_heap as dh
import time 
import sys

def showMap(length , blocklist = None, pathlist = None, m_map = None, old_path = None):
	if not( m_map is None):
		data = m_map[2]
		fig, ax = plt.subplots()
		ax.imshow(data, cmap=ListedColormap(['grey','white','black']))
	else:
		data = np.zeros((length, length))
		if not (blocklist is None):
			data = blocklist
		fig, ax = plt.subplots()
		ax.imshow(data, cmap=ListedColormap(['white','black']))
	ax.set_xticks(np.arange(-.5, length, 1), minor=True)
	ax.set_yticks(np.arange(-.5, length, 1), minor=True)
	ax.grid(which= 'minor', color='black', linestyle='-', linewidth=1)
	if pathlist != None:
		poltline(pathlist,'red')
	if old_path != None:
		poltline(old_path,'green')
	plt.show()

def poltlinec(A,c):
    x , y = zip(*A)
    line, = plt.plot(y,x,color=c)
    return line

def showmode_launch(length , blocklist = None, pathlist = None, m_map = None, old_path = None):
	data = m_map[2]
	fig, ax = plt.subplots()
	cmap = colors.ListedColormap(['grey','white','black'])
	bounds=[-0.5,0.5,1.5,2.5]
	norm = colors.BoundaryNorm(bounds, cmap.N)
	im = ax.imshow(data, interpolation='nearest', origin='lower',cmap=cmap, norm=norm )
	ax.set_xticks(np.arange(-.5, length, 1), minor=True)
	ax.set_yticks(np.arange(-.5, length, 1), minor=True)
	ax.grid(which= 'minor', color='black', linestyle='-', linewidth=1)
	oldline = poltlinec(old_path,'green')
	pathline = poltlinec(pathlist, 'red')
	plt.pause(1)
	return im, fig,oldline,pathline

def showmode_update(im, fig, oldline,pathline, m_map, alist, blist):
	im.set_data(m_map)
	x, y = zip(*alist)
	oldline.set_xdata(y)
	oldline.set_ydata(x)
	x, y = zip(*blist)
	pathline.set_xdata(y)
	pathline.set_ydata(x)
	plt.pause(1)
	fig.canvas.draw()

def A_Search_adp(map_length,blocklist, start, goal, show = False):
	# 0 search, 1 for g, 2 for blocked, 3 for pre_closed, 4 for current_closed,
	max_g = map_length * map_length
	current_cell = start 
	goal_cell = goal
	magic_map = np.zeros((5,map_length, map_length))
	magic_map[3] = np.zeros((map_length,map_length), dtype = bool)
	magic_map[4] = np.zeros((map_length,map_length), dtype = bool)
	magic_map[2][current_cell] = 1
	p_max_g = 0
	current_steps = 0 
	num_expend = 0 
	move_trace = [start]
	UpdataBlocks(magic_map,map_length,current_cell,blocklist)
	while current_cell != goal_cell:
		current_steps += 1 
		magic_map[1][current_cell] = 0
		magic_map[0][current_cell] = current_steps
		magic_map[1][goal_cell] = np.inf
		magic_map[0][goal_cell] = current_steps
		open_list = []
		open_dict = dict()
		from_dict = dict()
		magic_map[4] = np.zeros((map_length,map_length), dtype = bool)
		dh.insert(open_list, open_dict, Manhatton_value(current_cell,goal_cell), current_cell)
		while open_list and magic_map[1][goal_cell] > open_list[0]:
			c_cell = dh.pop(open_list, open_dict)
			magic_map[4][c_cell] = True
			num_expend += 1
			for n_cell in vaild_neighbor(c_cell,map_length):
				if magic_map[2][n_cell] != 2 :
					if magic_map[3][n_cell]:
						h_new = p_max_g - magic_map[1][n_cell]
					else:
						h_new = Manhatton_value(n_cell,goal_cell)
					if magic_map[0][n_cell] < current_steps:
						magic_map[1][n_cell] = np.inf
						magic_map[0][n_cell] = current_steps
					if magic_map[1][n_cell] > magic_map[1][c_cell] + 1:
						from_dict[n_cell] = c_cell
						new_g = magic_map[1][c_cell] + 1
						dh.insert(open_list, open_dict, (max_g*( new_g +  h_new) -  new_g) , n_cell )
						magic_map[1][n_cell] = new_g
		magic_map[3] = magic_map[4]
		p_max_g = magic_map[1][goal_cell]
		if not open_list:
			return None, None
		current_path = getPath(from_dict,current_cell,goal_cell)
		if show:
			if current_cell == start:
				plt.ion()
				im, fig,oldline,pathline = showmode_launch(map_length, m_map = magic_map, pathlist = current_path , old_path = move_trace)
				plt.show()
			else:
				showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
		for cell in current_path:
			if cell == current_cell:
				continue
			else:
				if magic_map[2][cell] != 2 :
					move_trace.append(cell)
					current_cell = cell
					UpdataBlocks( magic_map,map_length,current_cell,blocklist)
				else:
					break 
	if show:
		showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
	return move_trace,num_expend

def A_Search_Re_Backward_Tb(map_length,blocklist, start, goal, show = False):
	# 0 search, 1 for g, 2 for blocked, 3 for closed
	num_expend = 0 
	g_max = map_length*map_length
	current_cell = start 
	goal_cell = goal
	magic_map = np.zeros((4,map_length, map_length))
	magic_map[2][current_cell] = 1
	current_steps = 0 
	move_trace = [start]
	UpdataBlocks(magic_map,map_length,current_cell,blocklist)
	while current_cell != goal_cell:
		magic_map[3] = np.zeros((map_length,map_length))
		current_steps += 1 
		magic_map[1][current_cell] = np.inf
		magic_map[0][current_cell] = current_steps
		magic_map[1][goal_cell] = 0
		magic_map[0][goal_cell] = current_steps
		open_list = []
		open_dict = dict()
		from_dict = dict()
		dh.insert(open_list, open_dict, Manhatton_value(current_cell,goal_cell), goal_cell)
		while open_list and magic_map[1][current_cell] > open_list[0]:
			c_cell = dh.pop(open_list, open_dict)
			magic_map[3][c_cell] = 1
			num_expend += 1
			for n_cell in vaild_neighbor(c_cell,map_length):
				if magic_map[2][n_cell] != 2 :
					if magic_map[0][n_cell] < current_steps:
						magic_map[1][n_cell] = np.inf
						magic_map[0][n_cell] = current_steps
					if magic_map[1][n_cell] > magic_map[1][c_cell] + 1:
						magic_map[1][n_cell] = magic_map[1][c_cell] + 1
						from_dict[n_cell] = c_cell
						dh.insert(open_list, open_dict, g_max*(magic_map[1][n_cell] + Manhatton_value(n_cell,current_cell)) - magic_map[1][n_cell] , n_cell )
		if not open_list:
			#print("no rout")
			return None, None
		
		if show:
			current_path = getBackPath(from_dict,current_cell,goal_cell)
			if current_cell == start:
				plt.ion()
				im, fig,oldline,pathline = showmode_launch(map_length, m_map = magic_map, pathlist = current_path , old_path = move_trace)
				plt.show()
			else:
				showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
		while current_cell != goal_cell:
			cell = from_dict[current_cell]
			if magic_map[2][cell] != 2 :
				move_trace.append(cell)
				current_cell = cell
				UpdataBlocks( magic_map,map_length,current_cell,blocklist)
			else:
				break
	if show:
		showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path) 
	return move_trace, num_expend

def A_Search_Re_Forward_Tb(map_length,blocklist, start, goal, show = False):
	# 0 search, 1 for g, 2 for blocked, 
	num_expend = 0
	g_max = map_length* map_length
	current_cell = start 
	goal_cell = goal
	magic_map = np.zeros((4,map_length, map_length))
	magic_map[2][current_cell] = 1
	current_steps = 0 
	move_trace = [start]
	UpdataBlocks(magic_map,map_length,current_cell,blocklist)
	while current_cell != goal_cell:
		current_steps += 1 
		magic_map[1][current_cell] = 0
		magic_map[0][current_cell] = current_steps
		magic_map[1][goal_cell] = np.inf
		magic_map[0][goal_cell] = current_steps
		open_list = []
		open_dict = dict()
		from_dict = dict()
		dh.insert(open_list, open_dict, Manhatton_value(current_cell,goal_cell), current_cell)
		while open_list and magic_map[1][goal_cell] > open_list[0]:
			c_cell = dh.pop(open_list, open_dict)
			magic_map[3][c_cell] = 1
			num_expend += 1
			for n_cell in vaild_neighbor(c_cell,map_length):
				if  magic_map[2][n_cell] != 2 :
					if magic_map[0][n_cell] < current_steps:
						magic_map[1][n_cell] = np.inf
						magic_map[0][n_cell] = current_steps
					if magic_map[1][n_cell] > magic_map[1][c_cell] + 1:
						magic_map[1][n_cell] = magic_map[1][c_cell] + 1
						from_dict[n_cell] = c_cell
						dh.insert (open_list, open_dict, ( g_max*(magic_map[1][n_cell] + Manhatton_value(n_cell,goal_cell)) - magic_map[1][n_cell] )  , n_cell ) 
		if not open_list:
			#print("no rout")
			return None, None
		current_path = getPath(from_dict,current_cell,goal_cell)
		if show:
			if current_cell == start:
				plt.ion()
				im, fig,oldline,pathline = showmode_launch(map_length, m_map = magic_map, pathlist = current_path , old_path = move_trace)
				plt.show()
			else:
				showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
		#print(num_expend)
		for cell in current_path:
			if cell == current_cell:
				continue
			else:
				if magic_map[2][cell] != 2 :
					move_trace.append(cell)
					current_cell = cell
					UpdataBlocks( magic_map,map_length,current_cell,blocklist)
				else:
					break 
	if show:
		showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
	return move_trace, num_expend

def A_Search_Re_Forward_Ta(map_length,blocklist, start, goal, show = False):
	# 0 search, 1 for g, 2 for blocked, 
	num_expend = 0
	g_max = map_length* map_length
	current_cell = start 
	goal_cell = goal
	magic_map = np.zeros((4,map_length, map_length))
	magic_map[2][current_cell] = 1
	current_steps = 0 
	move_trace = [start]
	UpdataBlocks(magic_map,map_length,current_cell,blocklist)
	while current_cell != goal_cell:
		current_steps += 1 
		magic_map[1][current_cell] = 0
		magic_map[0][current_cell] = current_steps
		magic_map[1][goal_cell] = np.inf
		magic_map[0][goal_cell] = current_steps
		open_list = []
		open_dict = dict()
		from_dict = dict()
		dh.insert(open_list, open_dict, Manhatton_value(current_cell,goal_cell), current_cell)
		while open_list and magic_map[1][goal_cell] > open_list[0]:
			c_cell = dh.pop(open_list, open_dict)
			num_expend += 1
			for n_cell in vaild_neighbor(c_cell,map_length):
				if  magic_map[2][n_cell] != 2 :
					if magic_map[0][n_cell] < current_steps:
						magic_map[1][n_cell] = np.inf
						magic_map[0][n_cell] = current_steps
					if magic_map[1][n_cell] > magic_map[1][c_cell] + 1:
						magic_map[1][n_cell] = magic_map[1][c_cell] + 1
						from_dict[n_cell] = c_cell
						dh.insert (open_list, open_dict, ( g_max*(magic_map[1][n_cell] + Manhatton_value(n_cell,goal_cell)) + magic_map[1][n_cell] )  , n_cell ) 
		if not open_list:
			#print("no rout")
			return None, None
		current_path = getPath(from_dict,current_cell,goal_cell)
		if show:
			if current_cell == start:
				plt.ion()
				im, fig,oldline,pathline = showmode_launch(map_length, m_map = magic_map, pathlist = current_path , old_path = move_trace)
				plt.show()
			else:
				showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
		for cell in current_path:
			if cell == current_cell:
				continue
			else:
				if magic_map[2][cell] != 2 :
					move_trace.append(cell)
					current_cell = cell
					UpdataBlocks( magic_map,map_length,current_cell,blocklist)
				else:
					break 
	if show:
		showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
	return move_trace, num_expend

def A_Search_Re_Forward(map_length,blocklist, start, goal, show = False):
	# 0 search, 1 for g, 2 for blocked, 
	num_expend = 0
	current_cell = start 
	goal_cell = goal
	magic_map = np.zeros((3,map_length, map_length))
	magic_map[2][current_cell] = 1
	current_steps = 0 
	move_trace = [start]
	UpdataBlocks(magic_map,map_length,current_cell,blocklist)
	while current_cell != goal_cell:
		current_steps += 1 
		magic_map[1][current_cell] = 0
		magic_map[0][current_cell] = current_steps
		magic_map[1][goal_cell] = np.inf
		magic_map[0][goal_cell] = current_steps
		open_list = []
		open_dict = dict()
		from_dict = dict()
		dh.insert(open_list, open_dict, Manhatton_value(current_cell,goal_cell), current_cell)
		while open_list and magic_map[1][goal_cell] > open_list[0]:
			c_cell = dh.pop(open_list, open_dict)
			num_expend +=1
			for n_cell in vaild_neighbor(c_cell,map_length):
				if  magic_map[2][n_cell] != 2 :
					if magic_map[0][n_cell] < current_steps:
						magic_map[1][n_cell] = np.inf
						magic_map[0][n_cell] = current_steps
					if magic_map[1][n_cell] > magic_map[1][c_cell] + 1:
						magic_map[1][n_cell] = magic_map[1][c_cell] + 1
						from_dict[n_cell] = c_cell
						dh.insert(open_list, open_dict, (magic_map[1][n_cell] + Manhatton_value(n_cell,goal_cell)) , n_cell )
		if not open_list:
			#print("no rout")
			return None, None
		current_path = getPath(from_dict,current_cell,goal_cell)
		if show:
			
			if current_cell == start:
				plt.ion()
				im, fig,oldline,pathline = showmode_launch(map_length, m_map = magic_map, pathlist = current_path , old_path = move_trace)
				plt.show()
			else:
				showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
		for cell in current_path:
			if cell == current_cell:
				continue
			else:
				if magic_map[2][cell] != 2 :
					move_trace.append(cell)
					current_cell = cell
					UpdataBlocks( magic_map,map_length,current_cell,blocklist)
				else:
					break 
	if show:
		showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
	return move_trace,num_expend

def A_Search_Re_Backward(map_length,blocklist, start, goal, show = False):
	# 0 search, 1 for g, 2 for blocked, 3 for closed
	num_expend = 0
	current_cell = start 
	goal_cell = goal
	magic_map = np.zeros((3,map_length, map_length))
	magic_map[2][current_cell] = 1
	current_steps = 0 
	move_trace = [start]
	UpdataBlocks(magic_map,map_length,current_cell,blocklist)
	while current_cell != goal_cell:
		current_steps += 1 
		magic_map[1][current_cell] = np.inf
		magic_map[0][current_cell] = current_steps
		magic_map[1][goal_cell] = 0
		magic_map[0][goal_cell] = current_steps
		open_list = []
		open_dict = dict()
		from_dict = dict()
		dh.insert(open_list, open_dict, Manhatton_value(current_cell,goal_cell), goal_cell)
		while open_list and magic_map[1][current_cell] > open_list[0]:
			num_expend += 1
			c_cell = dh.pop(open_list, open_dict)
			for n_cell in vaild_neighbor(c_cell,map_length):
				if magic_map[2][n_cell] != 2 :
					if magic_map[0][n_cell] < current_steps:
						magic_map[1][n_cell] = np.inf
						magic_map[0][n_cell] = current_steps
					if magic_map[1][n_cell] > magic_map[1][c_cell] + 1:
						magic_map[1][n_cell] = magic_map[1][c_cell] + 1
						from_dict[n_cell] = c_cell
						dh.insert(open_list, open_dict,magic_map[1][n_cell] + Manhatton_value(n_cell,current_cell), n_cell )
		if not open_list:
			#print("no rout")
			return None, None
		
		if show:
			current_path = getBackPath(from_dict,current_cell,goal_cell)
			if current_cell == start:
				plt.ion()
				im, fig,oldline,pathline = showmode_launch(map_length, m_map = magic_map, pathlist = current_path , old_path = move_trace)
				plt.show()
			else:
				showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
		while current_cell != goal_cell:
			cell = from_dict[current_cell]
			if magic_map[2][cell] != 2 :
				move_trace.append(cell)
				current_cell = cell
				UpdataBlocks( magic_map,map_length,current_cell,blocklist)
			else:
				break 
	if show:
		showmode_update(im, fig, oldline,pathline, magic_map[2],  move_trace ,current_path)
	return move_trace, num_expend




if __name__ == '__main__':
	function_dict = { '1': A_Search_Re_Forward ,  '2': A_Search_Re_Forward_Tb, '3' :A_Search_Re_Forward_Ta , '4' : A_Search_Re_Backward_Tb , '5' :A_Search_adp }
	print("----------------Fast-Trajectory-Replanning-----------------")
	print("--Please choose Analyse Mode or Persentation Mode,")
	Mode = input("--1 for Analyse, 2 for Persentation: ")
	if Mode == '1':
		print("--Please choose Two of following algorithms")
		print("--1 for Repeated Forward  A* search without Tie break")
		print("--2 for Repeated Forward  A* search with Tie break in favour of larger g-values")
		print("--3 for Repeated Forward  A* search with Tie break in favour of smaller g-values")
		print("--4 for Repeated Backward A* search with Tie break in favour of larger g-values")
		print("--5 for Adaptive A* search with Tie break in favour of larger g-values")
		fun_a = function_dict[input("--Your first choice of algorithm: ")]
		fun_b = function_dict[input("--Your sencond choice of algorithm: ")]
		if input("--Do you want to use SEE(standard exprimental environment? (Yes or No)") == "Yes":
			l = 101
			start = (0,0)
			goal = (l-1,l-1)
			run_time = 50
		else:
			l = int(input("--Please enter map size: "))
			start = tuple(map(int,input("--Please enter start point(Format: int , int):  ").split(',')))
			goal = tuple(map(int,input("--Please enter start point(Format: int , int):  ").split(',')))
			run_time = int(input("--Please enter run times: "))
		BlockedMaps = []
		#start = (random.randint(0, l -1), random.randint(0, l -1))
		#goal = (random.randint(0, l -1), random.randint(0, l -1))
		print("--Map Size: ",l," * ",l)
		print("--start point:", start)
		print("--goal point :", goal)
		print("--MapGenerating Maps", end = '\r')
		for _ in range(run_time):
			BlockedMaps.append(BlockedMapGenerator(l, 0.3, start, goal))
		#showMap(l, blocklist = BlockedMaps[0])
		print("--Map Generation finished")


		A_n_list = []
		i = 1
		print("--Running maps using", str(fun_a))
		ftimes = []
		for bls in BlockedMaps:
			print("--Running Map #", i, end = '\r')
			s = time.time()
			move_trace,num_expend = fun_a(l, bls, start,goal)
			e = time.time()
			if move_trace:
				ftimes.append((e - s))
				A_n_list.append(num_expend)
			i += 1
		print("--Analyse of ", str(fun_a)," Finished")

		i = 1
		B_n_list = []
		print("--Running maps using", str(fun_b))
		btimes = []
		for bls in BlockedMaps:
			print("Run Map #", i, end = '\r')
			s = time.time()
			move_trace,num_expend = fun_b(l, bls, start,goal)
			e = time.time()
			if move_trace: 
				btimes.append((e - s))
				B_n_list.append(num_expend)
			i += 1
		if A_n_list is None:
			print("--Analyse Failed, All maps has no route-- :(")
		else:
			print("--Analyse of ", str(fun_b)," Finished")
			print("----------------------------------------------------------------------")
			print("--For", str(fun_a), "\n--Average solve time :", np.mean(ftimes),"\n--Average expanded cells:", np.mean(A_n_list))
			print("--For", str(fun_b), "\n--Average solve time :", np.mean(btimes),"\n--Average expanded cells:", np.mean(B_n_list))
	elif Mode == '2':
		print("--Please choose Two of following algorithms")
		print("--1 for Repeated Forward  A* search without Tie break")
		print("--2 for Repeated Forward  A* search with Tie break in favour of larger g-values")
		print("--3 for Repeated Forward  A* search with Tie break in favour of smaller g-values")
		print("--4 for Repeated Backward A* search with Tie break in favour of larger g-values")
		print("--5 for Adaptive A* search with Tie break in favour of larger g-values")
		fun_a = function_dict[input("--Your choice of algorithm: ")]
		if input("--Do you want to use SEE(standard exprimental environment? (Yes or No)") == "Yes":
			l = 30
			start = (0,0)
			goal = (l-1,l-1)
		else:
			l = int(input("--Please enter map size: "))
			start = tuple(map(int,input("--Please enter start point(Format: int , int):  ").split(',')))
			goal = tuple(map(int,input("--Please enter start point(Format: int , int):  ").split(',')))
		print("--Map Size: ",l," * ",l)
		print("--start point:", start)
		print("--goal point :", goal)
		#while True:
		print("--MapGenerating Map", end = '\r')
		while True:
			one_map = BlockedMapGenerator(l, 0.3, start, goal)
			p,a = A_Search_Re_Forward_Tb(l, one_map, start,goal)
			if not p is None:
				break
			#showMap(l, blocklist = one_map)
			#if input("Is the shown map good? (Yes or No)") == "Yes":
				#print("--Map Generation finished ")
				#break
		fun_a(l, one_map, start,goal, show =True)
		

	'''user_choice = 0
				if user_choice == 1:
					
				elif user_choice == 2:
					l = 101
					example_map = MapGenerator(l)
					#nt_path = A_Search_Re_Forward(101, example_map, (0,0), (100,100))
					#showMap(101,blocklist = example_map,   pathlist = nt_path)
					tb_path, num = A_Search_Re_Forward_Tb(l, example_map, (0,0), (100,100) )
					showMap(l, blocklist = example_map,  pathlist = tb_path)
					print(num)
				elif user_choice == 3:
					l = 101
					example_map = BlockedMapGenerator(l,0.3 )
			
					ns = time.time()
					nt_path = A_Search_adp(l, example_map, (0,0), (l-1,l-1))
					ne = time.time()
			
					ps = time.time()
					tb_path = A_Search_Re_Forward_Tb(l, example_map, (0,0), (l-1,l-1))
					pe = time.time()
			
					print("Normal--%s--" %(ne-ns), len(nt_path))
					print("Adapti--%s--" %(pe-ps), len(tb_path))
			
					showMap(l,blocklist = example_map,   pathlist = nt_path)
					showMap(l, blocklist = example_map,  pathlist = tb_path)'''

	
