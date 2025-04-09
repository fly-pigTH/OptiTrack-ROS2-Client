# OptiTrack-ROS2-Client


## ubuntu side

### modify

please change the server ip and port in src/optitrack_ros2_interface/optitrack_ros2_interface/pose_publisher.py to your own ip address:
```
SERVER_IP = '172.16.0.1'
SERVER_PORT = 1234
```

### install
```
cd src
colcon build
echo 'source '$PWD'/install/setup.bash' >> ~/.bashrc
bash
```
### run
```
ros2 run optitrack_ros2_interface pose_publisher
```

## windows side

### modify

please change the server ip and port in windows_side/RobotSensor.py to your own ip address:
```
SERVER_IP = '172.16.0.1'
SERVER_PORT = 1234
```

### run

#### open optitrack stream:

1. open motive
2. setting->streaming: open natnet streaming
3. select 'multicast' and 'loopback'

#### start program
```
python windows_side/RobotSensor.py
```