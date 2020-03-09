## launch tutorial

### launch nodes
```
ros2 launch tutorial_launch nodes.launch.py
```

### nodes with remapped namespace, node name and topic name
```
ros2 launch tutorial_launch nodes_with_remapping.launch.py
```

### nodes with loading parameters that is defined launch file
```
ros2 launch tutorial_launch nodes_with_param.launch.py
```

### nodes with loading parameter file that is defined yaml file
```
ros2 launch tutorial_launch nodes_with_param_file.launch.py
```

### launching other launch file
```
ros2 launch tutorial_launch launch_file.launch.py
```

### launch composition node
```
ros2 launch tutorial_launch composition_nodes.launch.py
```


## reference
- https://index.ros.org/doc/ros2/Tutorials/Composition/
- https://roscon.ros.org/2018/presentations/ROSCon2018_launch.pdf
- https://github.com/ros2/launch_ros/blob/master/launch_ros/examples/lifecycle_pub_sub_launch.py
