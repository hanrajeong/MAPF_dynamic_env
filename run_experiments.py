#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from weightedSIPP import weightedSIPP
from sipp_algorithm import SIPP
from astar import AStar
from weightedSIPP import weightedSIPP
from anytimesipp import Anytime_SIPP
from visualize import Animation
from single_agent_planner import get_sum_of_cost
from math import inf
from random_agents_generator import random_generator

SOLVER = "Anytime_SIPP"

def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # rows lines with the map
    # print(rows, columns)
    safe_intervals = [[[] for _ in range(columns)] for _ in range(rows)]
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for c in range(columns):
            # print(len(line), r, c)
            if line[c] == '@':
                my_map[-1].append(True)
            elif line[c] == '.':
                my_map[-1].append(False)
                # print(len(self.safe_intervals), len(self.safe_intervals[0]))
                safe_intervals[r][c].append(safe_interval())
    map_object = Map()
    map_object.making_map(rows, columns, safe_intervals)
    # agents
    line = f.readline()
    num_agents = int(line)
    # agents lines with the start/goal positions
    starts = []
    goals = []
    agents = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        agent = obstacles(sx, sy, gx, gy)
        agents.append(agent)
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, map_object, starts, goals, agents

def import_obstacles(mapname, filename, my_map, starts, goals, agents):
    rows = len(my_map)
    columns = len(my_map[0])
    if filename == "random":
        filename = random_generator(mapname, my_map, rows, columns, starts[0], goals[0])
    # print("herehere")
    obstacle_list = []
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    with f:
        for line in f:
            sx, sy, gx, gy = [int(x) for x in line.split('\t')]
            if my_map[sx][sy] or my_map[gx][gy]:
                # print("obstacle is here", sx, sy, gx, gy)
                continue
            # print(sx, sy, gx, gy)
            starts.append((sx, sy))
            goals.append((gx, gy))
            obstacle = obstacles(sx, sy, gx, gy)
            agents.append(obstacle)
            obstacle_list.append(obstacle)
    return starts, goals, obstacle_list, agents


class obstacles:
    def __init__(self, start_x, start_y, goal_x, goal_y):
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y


class safe_interval:

    def __init__(self, start_time=0, end_time=inf):
        self.start_time = start_time
        self.end_time = end_time

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)

def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)

class Map:
    def __init__(self):
        self.columns = 0
        self.rows = 0
        self.safe_intervals = [[[] for _ in range(self.columns)] for _ in range(self.rows)]
        self.agents = []
    
    def making_map(self, rows, columns, safe_intervals):
        self.rows = rows
        self.columns = columns
        self.safe_intervals = safe_intervals

    def movement(self, i, j):
        movements = []
        dir = [[1, 0], [-1, 0], [0, -1], [0, 1], [0, 0]]
        for d in dir:
            if (0 <= i + d[0] < self.rows) and (0 <= j + d[1] < self.columns) and len(self.safe_intervals[i + d[0]][j + d[1]]) > 0 :
                # print(len(self.safe_intervals[i + d[0]][j + d[1]]))
                movements.append((i + d[0], j + d[1]))
        return movements

    def successors(self, node):
        # successors = 0
        successors = []
        for movement_ in self.movement(node.r, node.c):
            # m_time = time to execute m
            m_time = 1
            # start_t = time(s) + m_time
            start_t = node.g + m_time
            # end_t = endTime(intervale(s)) + m_time
            # print(node.r, node.c, node.interval)
            # print(self.safe_intervals[node.r][node.c])
            end_t = self.safe_intervals[node.r][node.c][node.interval].end_time + 1
            # for each safe interval i in cfg
            i, j = movement_
            # if startTime(i) > end_t or endTime(i) < start_t
            for idx, safe_interval in enumerate(self.safe_intervals[i][j]):
                if safe_interval.start_time > end_t or safe_interval.end_time < start_t:
                    continue
                # t = earliest arrival time at cfg during interval i with no collision
                time = max(start_t, safe_interval.start_time)
                if time == safe_interval.start_time:
                    for agent in self.agents:
                        if time < len(agent) and agent[time-1] == (i, j) and agent[time] == (node.r, node.c):
                            time = inf
                            break
                # if t does not exist
                # continue
                if time > min(end_t, safe_interval.end_time):
                    continue
                # s' = state of configuration cfg with interval i and time t
                # insert s' into successors
                successors.append((i, j, idx, time))
        return successors

    def getting_paths(self, path):
        paths = []
        for i in range(1, len(path)):
            current = path[i]
            previous = path[i-1]
            for _ in range(current.g - previous.g):
                paths.append((previous.r, previous.c))
        paths.append((path[-1].r, path[-1].c))
        return paths

    def dynamic_environment(self, obstacle_paths):
        # print("dynamic")
        for o in obstacle_paths:
            paths = self.getting_paths(o)
            self.agents.append(paths)
            obstacle = [[[] for _ in range(self.columns)] for _ in range(self.rows)]
            # Made it as the set, to check the duplicate
            updated_map = set()
            for idx, (i, j) in enumerate(paths):
                updated_map.add((i, j))
                obstacle[i][j].append(idx)
            for i, j in updated_map:
                q = 0
                updated_safe_intervals = []
                for interval in self.safe_intervals[i][j]:
                    while q < len(obstacle[i][j]) and obstacle[i][j][q] < interval.start_time:
                        q+=1
                    
                    if q >= len(obstacle[i][j]):
                        updated_safe_intervals.append(interval)
                        continue

                    current_start_time = interval.start_time
                    while q < len(obstacle[i][j]) and obstacle[i][j][q] <= interval.end_time:
                        current_end_time = obstacle[i][j][q] - 1
                        if current_start_time <= current_end_time:
                            updated_safe_intervals.append(safe_interval(current_start_time, current_end_time))

                        current_start_time = current_end_time + 3
                        q+=1
                    
                    if current_start_time <= interval.end_time:
                        updated_safe_intervals.append(safe_interval(current_start_time, interval.end_time))

                self.safe_intervals[i][j] = updated_safe_intervals
                




if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--obstacle', type=str, default = None,
                        help = 'obstacle file containing obstacles')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized, AnytimeSIPP}), defaults to ' + str(SOLVER))
    parser.add_argument('--gif', type=str, default=None,
                        help = 'The name of the gif file that you want to save as')
    args = parser.parse_args()


    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, map_object, starts, goals, agents = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)
        if args.obstacle is not None:
            starts, goals, obstacle_list, agents = import_obstacles(args.instance, args.obstacle, my_map, starts, goals, agents)

        if args.solver == "AnytimeSIPP":
            print("***Anytime SIPP***")
            solver = Anytime_SIPP(map_object, agents)
            paths = solver.find_solution()
        elif args.solver == "SIPP":
            print("***SIPP***")
            solver = SIPP(map_object, agents)
            paths = solver.find_solution()
        elif args.solver == "AStar":
            print("***AStar***")
            solver = AStar(map_object, agents)
            paths = solver.find_solution()
        elif args.solver == "weightedSIPP":
            print("***weightedSIPP***")
            solver = weightedSIPP(map_object, agents)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")
        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))


        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            if args.gif:
                # print("save")
                animation.save('./' + args.gif + '.gif', 6)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
