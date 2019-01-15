'''
#by Huaheng Yu (hy240), Maisy Chung (mlc340), Yi Lyu (yl866)
heap module custmized for RA*
strcut:
	heap_list : contain all f_values
	heap_dict : retrive cells with that f_values
				{ f : [ (x,y), (x,y), ... ] , f : [(x,y), (x,y), ...]    }
				 f_value   list of cell with same f 
'''
import random

def minfy(heap_list, i):
	length = len(heap_list)
	l = i*2 + 1
	r = i*2 + 2
	s = i
	if l < length - 1 and heap_list[l] < heap_list[i]:
		s = l 
	if r < length - 1 and heap_list[r] < heap_list[s]:
		s = r
	if s != i:
		heap_list[s] , heap_list[i] = heap_list[i] , heap_list[s]
		minfy(heap_list, s)


def pop(heap_list, heap_dict):
	min_f = heap_list[0]
	cell = heap_dict[min_f].pop(random.randrange(len(heap_dict[min_f]))) 
	if len(heap_dict[min_f]) == 0:
		del heap_dict[min_f]
		heap_list[0] = heap_list[-1]
		heap_list.pop()
		minfy(heap_list, 0)  
	return cell

def insert(heap_list, heap_dict, f_value, cell):
	if f_value in heap_list:
		heap_dict[f_value].append(cell)
	else:
		heap_dict[f_value] = [cell]
		p = len(heap_list) 
		heap_list.append(None)
		while p > 0 and f_value < heap_list[(p - 1)//2]:
			heap_list[p] = heap_list[(p - 1)//2]
			p = (p - 1)//2
		heap_list[p] = f_value
