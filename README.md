MAPF_dynamic_env 
  
This is for the group project in SFU, CMPT417 course in Summer semester 2022.  
The team member : Hanra Jeong, Jooyoung Julia Lee, Fitz Laddaran. 

Implementation environment
```
Conda : 4.14.0 (for the virtual environment setting)
Python version : Python 3.9.13
ffmpeg version : 5.1 (for creating gif file)
matplotlib version : 3.5.1
used computer : Apple M1 Pro 2021
-    M1 macOS Monterey Version 12.4
-    10-core CPU with 8 performance cores and 2 efficiency cores
-    16-core GPU
-    16-core Neural Engine
-    Memory 16 GB
-    Flash Storage 1TB
```
With this setting, the code can be run by the below structure of command line.  
```python run_experiments.py [--instance map_file] [--obstacle obstacle] [--solver solving algorithm]``` 

For the arguments:
```
--instance [map_file_name]    : The name of the map instance file
--obstacle [input]            : random – for generating the obstacle randomly
                              : obstacle_file_name – for running the submitted obstacle file
--solver algorithm_name       : AStar – for running A* algorithm
                              : SIPP – for running SIPP algorithm
                              : weightedSIPP – for running WSIPPd algorithm
                              : AnytimeSIPP – for running AnytimeSIPP algorithm
```. 
To see the more descriptive explanation with argument:
``` Python run_experiments.py –help ```

Abstract
With an interest in the autonomy of modern automobiles and intelligent robots within a dynamic environment, Safe Interval Path Planning (SIPP) is proposed to find the path in such condition. In this paper, we implemented SIPP, Weighted SIPP with Duplicate States (WSIPPd) and Anytime SIPP algorithm. We compare the different results of these and A* algorithm which is known optimal solution. 

Introduction  
As we implemented in the individual project with A*, Prioritized, CBS algorithms, Multi-agent path finding (MAPF) is important challenge for many applications under static environment. However, in a real-life, for autonomous car, robotic vacuum, or the automated warehousing robots to safely achieve their goals, they need to navigate from the starting to the goal state in the presence of other moving obstacles like human or other robots. For adopting the dynamic obstacles, we need to predict their paths and locations and adopt this to find the optimal path of target agent.  
Safe Interval Path Planning (SIPP) algorithm was proposed to solve this [1] which is known as complete and returning optimal solutions. However, for this SIPP algorithm, to get the solution faster, we need to trade off the solution optimality. [2] To minimize this and get the better optimal solution, a bounded-suboptimal SIPP algorithm is introduced. [3]
