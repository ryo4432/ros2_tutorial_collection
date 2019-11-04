# tutorial_publisher

## how to use

```
ros2 run tutorial_publisher talker
```
## topic name remapping

```
ros2 run tutorial_publisher talker topic:=<new topic cname>
```

## node name remapping

```
ros2 run tutorial_publisher talker __ns:=/ __node:=<new node name>
```
or
```
ros2 run tutorial_publisher talker talker:__node:=<new node name>
```
ref:https://index.ros.org/doc/ros2/Tutorials/Node-arguments/#name-remapping
