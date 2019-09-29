# run service server

```
ros2 run tutorial_service service_server
```

# service call from terminal

```
ros2 service call /server_node tutorial_msgs/SetMessage "{message: 'Hello Service!'}"
```
# service call in synchronous

```
ros2 run tutorial_service service_sync_client
```

# service call in asynchronous

```
ros2 run tutorial_service service_async_client
```
