from cmath import inf
from multiprocessing.spawn import import_main_path
import time as timer
from single_agent_planner import get_sum_of_cost
import math
import heapq
from property import Open, Closed, Node

def manhattan(x1, y1, x2, y2):
    d1 = abs(x1 - x2)
    d2 = abs(y1 - y2)
    return d1 + d2
    
class Anytime_SIPP:
    def __init__(self, map_object, agents):
        self.heuristic_function = manhattan
        self.goal_node = None
        self.path_exist = False
        self.starting_epsilon = 15
        self.path_list = []
        self.path_object_list = []
        self.my_map = map_object
        self.agents = agents
        self.total_cost = 0

    def find_solution(self):
        self.start_time = timer.time()
        founded_paths = []
        founded_paths_object = []
        for agent in self.agents:
            path_finding_result, sgoal = self.path_finding(self.my_map, agent.start_x, agent.start_y, agent.goal_x, agent.goal_y)
            if not path_finding_result:
                self.publish_paths([], False, [])
                return
            p, path_list, cost = self.paths(sgoal)
            # for jj in range(len(p)):
            #     print(p[jj].r, p[jj].c)
            #     print(path_list[jj])
            # print(path_list)
            # print(type(p))
            self.total_cost += cost
            self.my_map.dynamic_environment([p])
            founded_paths_object.append(p)
            founded_paths.append(path_list)
        self.publish_paths(founded_paths, True, founded_paths_object)

        if self.path_exist:
            print("\n Found a solution! \n")
            CPU_time = timer.time() - self.start_time
            print("CPU time (s):    {:.2f}".format(CPU_time))
            print("Sum of costs:    {}".format(get_sum_of_cost(self.path_list)))
        # print((self.path_list[0]))
        # print(len(self.path_list))
        return self.path_list

    def publish_paths(self, paths, path_exist, path_object_list):
        self.path_object_list = path_object_list
        self.path_exist = path_exist
        self.path_list = paths

    def publish(self, sgoal : Node, path_exist = True):
        self.goal_node = sgoal
        self.path_exist = path_exist
    
    def paths(self, sgoal : Node):
        cost = sgoal.g
        current = sgoal
        p = []
        path_list = []
        while current.parent:
            p.append(current)
            path_list.append((current.r, current.c))
            current = current.parent
        p.append(current)
        path_list.append((current.r, current.c))
        return p[::-1], path_list[::-1], cost

    def path_finding(self, my_map, start_x, start_y, goal_x, goal_y):
        #g(sstart) = 0
        sstart = Node(start_x, start_y, g = 0, h = self.starting_epsilon * self.heuristic_function(start_x, start_y, goal_x, goal_y))

        # OPEN = CLOSED = INCONS = 0
        OPEN = Open()
        CLOSED = Closed()
        INCONS = []
        # insert sstart into OPEN with epsilon * h(sstart)
        OPEN.add_node(sstart)
        #improve path
        # print("here1")
        OPEN, CLOSED, INCONS, sgoal = self.improve_path(OPEN, CLOSED, INCONS,  my_map, self.starting_epsilon, goal_x, goal_y)
        # epsilon' = min(epsilon, g(sgoal)/min OPEN U INCONS (g(s) + h(s)))
        minOpen = OPEN.get_minimum()
        minIncons = inf
        for nodeI in INCONS:
            minIncons = min(minIncons, nodeI.g + nodeI.h)

        if sgoal is not None:
            epsilon = min(self.starting_epsilon, sgoal.g/min(minOpen, minIncons))
            self.publish(sgoal)
        else:
            return self.path_exist, self.goal_node
        # while epsilon' > 1 do
        # decrease epsilon
        # Update the priorities for all s in OPEN according to f(s)
        while epsilon > 1:
            epsilon = epsilon / 2
            for node in INCONS:
                OPEN.add_node(node)
            OPEN.add_node(Node(start_x, start_y, h = epsilon * self.heuristic_function(start_x, start_y, goal_x, goal_y)))
            CLOSED = Closed()
            # print("here2")
            OPEN, CLOSED, INCONS, sgoal = self.improve_path(OPEN, CLOSED, INCONS, my_map, epsilon, goal_x, goal_y)
            minOpen = OPEN.get_minimum()
            for nodeI in INCONS:
                minIncons = min(minIncons, nodeI.g + nodeI.h)
            epsilon = min(epsilon, sgoal.g / min(minOpen, minIncons))
            self.publish(sgoal)
            # CLOSED = 0
            # ImprovePath()
            # 0 = min(g(sgoal)= mins2OPEN[INCONS(g(s) + h(s)))
            # publish current "0-sub-optimal solution
        return self.path_exist, self.goal_node


    def improve_path(self, OPEN, CLOSED, INCONS, my_map, epsilon, goal_x, goal_y):
        # while f(sgoal) > min OPEN(f(s)) do
        # remove s with the smallest f(s) from OPEN
        while not OPEN.is_empty():
            # get s with the smallest f(s) from OPEN
            s = OPEN.smallest_node()
            # if found node is the goal node, return
            if s.r == goal_x and s.c == goal_y:
                return OPEN, CLOSED, INCONS, s
            # CLOSED = CLOSED U {s}
            CLOSED.add_node(s)
            # if O(s) = true then
            # Opt = {true, false}
            # else
            # Opt = {false}
            for i, j, idx, time in my_map.successors(s):
                if s.opt:
                    opts = [True, False]
                else:
                    opts = [False]
                # for all o in opt do
                for o in opts:
                    # s' = {x', i, o}
                    # print(s.r, s.c)
                        next_s = Node(i, j, g = time, interval = idx, opt = o, parent = s)
                        # if s' was not visited before then
                        # f(s') = g(s') = inf
                        if not CLOSED.exist(next_s):
                            # if O(s') then
                            # f(s') = epsilon * (g(s') + h(s'))
                            # else
                            # f(s') = g(s') + epsilon * h(s')
                            if o:
                                next_s.f = epsilon * (time + self.heuristic_function(i, j, goal_x, goal_y))
                            else:
                                next_s.f = time + epsilon * self.heuristic_function(i, j, goal_x, goal_y)
                            # insert s' into OPEN with f(s')
                            OPEN.add_node(next_s)
                        else:
                            closedNode = CLOSED.nodes[next_s.info()]
                        # end if
                        # if g(s') > t then
                            if closedNode.g > time:
                            # g(s') = t
                                closedNode.g = time
                                closedNode.opt = o
                                closedNode.parent = s
                                #insert s' into INCONS
                                INCONS.append(closedNode)
                                CLOSED.nodes[next_s.info()] = closedNode
        return OPEN, CLOSED, INCONS, None