# diver_pose_ros_dummy
This is a package to check receiving the camera images and sending a result by TCP

# Setup
Change the directory
```bash
cd {ros worksapce}/src/underwater_drone_ros/diver_pose_ros_dummy
```

Install the package
```bash
pip install -r requirements.txt
pip install -e .
```

Change the directory
```bash
cd {ros worksapce}
```

build the package
```bash
catkin_make
source ./devel/setup.bash
```

# Execute
Follow the bellow step at each terminal

1. run the roscore
```bash
roscore
```

2. run the camera
```bash
roslaunch spinnaker_camera_driver stereo.launch
```

3. run the pipeline
```bash
rosrun diver_pose_ros_dummy pipeline.py
```

4. run the server
Because of server socket path, this should be run at below path.
```bash
cd {ros worksapce}/src/underwater_drone_ros/diver_pose_ros_dummy
rosrun diver_pose_ros_dummy recog_uds_server.py
```

5. run the client
Because of server socket path, this should be run at below path.
```bash
cd {ros worksapce}/src/underwater_drone_ros/diver_pose_ros_dummy
rosrun diver_pose_ros_dummy jenti_uds_client.py
```

When we excute all of them, server and client will give and take
the massage.
