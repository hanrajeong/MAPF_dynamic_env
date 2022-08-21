from property import Open, Closed, Node
from cmath import inf
from multiprocessing.spawn import import_main_path
import time as timer
from single_agent_planner import get_sum_of_cost

def manhattan(x1, y1, x2, y2):
    d1 = abs(x1 - x2)
    d2 = abs(y1 - y2)
    return d1 + d2
# Referred paper is linked below:
# https://www.aaai.org/AAAI22Papers/AAAI-2389.LiJ.pdf
class SIPP:
    def __init__(self, map_object, agents):
        self.goal_node = None
        self.path_exist = False
        self.my_map = map_object
        self.agents = agents
        self.heuristic_function = manhattan
        self.total_cost = 0
        # self.target_path = []

    def find_solution(self):
        self.start_time = timer.time()
        founded_paths = []
        founded_paths_object = []
        # print(self.agents[0].start_x, self.agents[0].start_y)
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
            # if agent == self.agents[0]:
            #     self.target_path.append(path_list)
        self.publish_paths(founded_paths, True, founded_paths_object)

        if self.path_exist:
            print("\n Found a solution! \n")
            CPU_time = timer.time() - self.start_time
            print("CPU time (s):    {:.2f}".format(CPU_time))
            print("Sum of costs:    {}".format(get_sum_of_cost(self.path_list)))
            # print("Cost of target:  {}".format(len(self.target_path)))
        # print((self.path_list[0]))
        # print(len(self.path_list))
        return self.path_list

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

    def publish_paths(self, paths, path_exist, path_object_list):
        self.path_object_list = path_object_list
        self.path_exist = path_exist
        self.path_list = paths

    def path_finding(self, my_map, start_x, start_y, goal_x, goal_y):
        #g(sstart) = 0
        sstart = Node(start_x, start_y, g = 0, h = self.heuristic_function(start_x, start_y, goal_x, goal_y))

        # OPEN = 0
        OPEN = Open()
        CLOSED = Closed()
        # insert sstart into OPEN with f(sstart) = h(sstart)
        # for here since f = h + g and g = 0, f = h
        OPEN.add_node(sstart)
        # while sgoal is not expanded
        # this is checked below with if CLOSED.exist(next_s) : continue
        while not OPEN.is_empty():
           # remove s with the smallest f-value from OPEN
            s = OPEN.smallest_node()
            # CLOSED = CLOSED U {s}
            CLOSED.add_node(s)
            # if found node is the goal node, return
            if s.r == goal_x and s.c == goal_y:
                self.publish(s)
                return self.path_exist, self.goal_node
            # successors = getSuccessors(s);
            # for each s' in successors
            for i, j, idx, time in my_map.successors(s):
                h = self.heuristic_function(i, j, goal_x, goal_y)
                # s' = {x', i, o}
                next_s = Node(i, j, g=time, h=h, interval=idx, parent=s)
                # if s' was not visited before then
                # f(s') = g(s') = inf
                if CLOSED.exist(next_s):
                    continue
                # insert s' into OPEN with f(s')
                OPEN.add_node(next_s)

        return self.path_exist, self.goal_node


    def publish(self, sgoal : Node, path_exist = True):
        self.goal_node = sgoal
        self.path_exist = path_exist