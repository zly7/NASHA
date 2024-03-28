```
                      .__   __.      ___           _______. __    __       ___      
                      |  \ |  |     /   \         /       ||  |  |  |     /   \     
                      |   \|  |    /  ^  \       |   (----`|  |__|  |    /  ^  \    
                      |  . `  |   /  /_\  \       \   \    |   __   |   /  /_\  \   
                      |  |\   |  /  _____  \  .----)   |   |  |  |  |  /  _____  \  
                      |__| \__| /__/     \__\ |_______/    |__|  |__| /__/     \__\ 
```
# NASHA:Narrow Pair Splitting Hybrid A* algrothm
## What's it
A Long-Distance path planning algorithm for narrow environments for ackerman vehicle.
## How To Run
The codebase has been compiled and run-tested on both x86 and ARM platforms with Ubuntu 18 and Ubuntu 20. Please use ROS1 for the ROS version. Follow the process below to compile and run:

``` bash
catkin_make \
&& source devel/setup.bash \
&& roslaunch hybrid_astar xxx.launch
```

In this command, xxx.launch should be replaced with the launch script you wish to execute. The example launch includes both a version with visualization and one without visualization. 
