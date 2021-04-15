# xvamr_sim

##  Hi

just create for conveninet


## Prerequisites

- scipy

## Usage
```
roslaunch amr_gazebo amr_world.launch 
rosrun amr_control line_follower.py
rostopic pub /follow_cmd std_msgs/Int8 "data: 1" 
```

### Gazebo

copy /models to ~/.gazebo




