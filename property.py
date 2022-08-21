from cmath import inf
from multiprocessing.spawn import import_main_path
import heapq

class Node:
    def __init__(self, r, c, g=0, h=0, f=None, parent=None, opt = False, interval=0):
        self.r = r
        self.c = c
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + self.h
        else:
            self.f = f
        self.parent = parent
        self.opt = opt
        self.interval = interval
    # https://www.pythonpool.com/python-__lt__/
    def __lt__(self, comparison):
        return self.f < comparison.f
    # This is to store and check the duplicate
    # because for g and h values, they are irrelavant to check the duplicate, so I need to make the data set that doesn't include g, h, f
    def info(self):
        return self.r, self.c, self.interval, self.opt

class Open:
    def __init__(self):
        self.node_data = []
        self.node_dict = {}
    
    def is_empty(self):
        return len(self.node_dict) == 0
    
    def add_node(self, node: Node, *args):
        # get the information of given node
        # get r, c, interval, opt
        info = node.info()
        # if the given node is existed
        # we need to update
        if info in self.node_dict:
            # find the existed node information
            current = self.node_dict[info]
            # if the current.g <= node. g, if means it takes more time by using node
            # therefore, just return, and stop
            if current.g <= node.g:
                return
        # if it is much cheaper to use given node
        # update the existing node with the given node
        self.node_dict[info] = node
        # push it to the array, node_data
        # at this time we use heap, to make it as priority queue
        heapq.heappush(self.node_data, node)

    def smallest_node(self):
        while True:
            # As we make it as priority queue,
            # get the cheapest = best node by heappop
            smallest = self.node_data[0]
            heapq.heappop(self.node_data)
            # Keep finding the node, to find the best node which already exist in the dictionary
            # if the chosen best node is already in the node_dict,
            if smallest.info() in self.node_dict:
                break
        # then, delete the node from the dictionary
        del self.node_dict[smallest.info()]
        # and then return the node
        return smallest

    def get_minimum(self):
        minOpen = inf
        for node in self.node_dict.values():
            minOpen = min(minOpen, node.g + node.h)
        return minOpen

    def exist(self, node: Node):
        return node.info() in self.node_dict

class Closed:
    def __init__(self):
        self.nodes = {}

    def add_node(self, node: Node, *args):
        self.nodes[node.info()] = node

    def exist(self, node: Node):
        return node.info() in self.nodes

    def get_minimum(self):
        minOpen = inf
        for node in self.nodes.values():
            minOpen = min(minOpen, node.g + node.h)
        return minOpen