# Project 3 Phase 2

### Name:    Smit Dumore
### UID :    119080641
### Dir ID : smitd

### Name:    Kiran Patil
### UID :    119398364
### Dir ID : kpatil27
# -------------------------

## Dependencies
1. python3
2. numpy
3. cv2
4. time
5. math
6. ros1
7. geometry_msgs
8. turtle bot simulation (https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 

## Github link
https://github.com/smitdumore/hybrid-astar

## video output
- Part 01 - https://www.youtube.com/watch?v=nvMFFGCs6NM
- Part 02 - https://www.youtube.com/watch?v=Ma9yxwXi9zc

## How to run code for PART 01
```
cd your_ros1_ws/src
git clone https://github.com/smitdumore/hybrid-astar.git
cd hybrid-astar/hybrid_Astar/src
python3 part01.py
```

## How to run code for PART 02
```
cd your_ros1_ws/src
git clone https://github.com/smitdumore/hybrid-astar.git
cd ..
catkin_make
source devel/setup.bash

roslaunch hybrid_Astar a_star.launch
```

## Sample input for Part 01

Enter start x coordinate: 30
Enter start y coordinate: 30
Enter start orientation: 0
Enter goal x coordinate: 550
Enter goal y coordinate: 30
enter clearance: 5
enter RPM1: 5
enter RPM2: 5

## Sample input for Part 02