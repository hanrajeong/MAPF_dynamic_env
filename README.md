#MAPF_dynamic_env 
  
###This is for the group project in SFU, CMPT417 course in Summer semester 2022.  
###The team member : Hanra Jeong, Jooyoung Julia Lee, Fitz Laddaran. 

##Implementation environment
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
##Command line
With this setting, the code can be run by the below structure of command line.  
```python run_experiments.py [--instance map_file] [--obstacle obstacle] [--solver solving algorithm]``` 
###For the arguments:
```
--instance [map_file_name]    : The name of the map instance file
--obstacle [input]            : random – for generating the obstacle randomly
                              : obstacle_file_name – for running the submitted obstacle file
--solver algorithm_name       : AStar – for running A* algorithm
                              : SIPP – for running SIPP algorithm
                              : weightedSIPP – for running WSIPPd algorithm
                              : AnytimeSIPP – for running AnytimeSIPP algorithm
``` 
To see the more descriptive explanation with argument:
``` Python run_experiments.py –help ```
