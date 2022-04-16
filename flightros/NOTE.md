# Flightmare + OMPL => FOMPL Planner
## My custom files and folders
I create a custom node: `fompl_planner` and hence a custom launch file `fompl.launch`
- /flightmare/flightros/include/fompl/
- /flightmare/flightros/src/fompl/
## How to run FOMPL Planner
**In terminal 1**
``` 
catkin_make
source devel/setup.bash
roslaunch flightros fompl.launch
```
**In terminal 2**
```
// NOTE: Right now publishing current and goal coordinates only trigger the plan() function, I'm still using random start and goal coordinates because of some bugs
rostopic pub /fompl_planner/current_position geometry_msgs/Point [TAB][TAB]
rostopic pub /fompl_planner/goal_position geometry_msgs/Point [TAB][TAB]
```

**In terminal 3**
```
rviz
```