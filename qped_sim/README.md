jazzy# 🚧🚧🚧🚧🚧🚧🚧🚧 
## 🚨 This is a branch in development! 🚨

```
cd $(QPED_WSPACE)/src
git clone --recurse-submodules git@gitlab.com:asantamarianavarro/code/projects/triffid/qped_sim.git
cd qped_sim
git lfs pull
```

## 👷 Clone dependencies and install their requirements:
```
git submodule init && git submodule update --remote
```

```
sudo apt install ros-jazzy-navigation2\
				ros-jazzy-nav2-bringup\
				ros-jazzy-control-msgs
```

```
sudo apt install ros-humble-navigation2\
				ros-humble-nav2-bringup\
				ros-humble-control-msgs
```
### LIO-SAM dependencies
```
sudo apt install ros-jazzy-perception-pcl \
  	   ros-jazzy-pcl-msgs \
  	   ros-jazzy-vision-opencv \
  	   ros-jazzy-xacro \
       ros-jazzy-octomap-mapping 

# Add GTSAM-PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### LIORF dependencies
```
sudo apt install libgeographic-dev
```

### Plansys dependency
```
sudo apt install ros-jazzy-plansys2-*\
        ros-jazzy-test-msgs
```

## 👷 Run code in development

**Terminal 1:**
Boot up simulation:
```
ros2 launch qped_bringup b2_bringup.launch.py
```

**Terminal 2:**
Instantiate Navigation stack + slam
```
ros2 launch qped_nav qped_nav.launch.py
```

**Terminal 3:**
Instantiate planning server
```
ros2 launch qped_plan qped_move_server_example.launch.py
```

**Terminal 4:**
Perform planning example via action calling
```
ros2 action  send_goal /nav_2_pose nav2_msgs/action/NavigateToPose "pose:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  pose:
    position:
      x: 0.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
behavior_tree: ''" 
```

**(optional) Terminal 3:**
Instantiate planning server
```
ros2 launch qped_plan qped_patrol_example.launch.py
```

**(optional) Terminal 4:**
Perform "hard-coded" patrolling:
```
ros2 run qped_plan patrolling_controller_node
```

👷 All topics can be visualized in RViz. For octomap visualization ensure you have plugin *octomap_rviz_plugins* installed and with Rviz you listen to ```/octomap_full_published```'s OccupancyGrid topic.

👷 Octomap is published once every 5 seconds.

# 🚧🚧🚧🚧🚧🚧🚧🚧


# Simulation of the Boston Dynamics Spot Robot using ROS2 and Ignition Gazebo (Fortress) on a Lunar Surface

This package simulates the Quadruped (qped) robot in Ignition Gazebo Fortress. For the platform control, we adapt [champ](https://github.com/chvmp/champ).

## Package Information
- `qped_bringup`: includes the controller yaml files and launch files
- `qped_description`: contains meshes model configs for three robots (inluding Spot)
- `qped_gazebo`: contains world/environment configs
- `champ`: includes configurations for the quadruped's kinematics/movement
- `champ_base`: includes configurations for the quadruped's specific state and controllers
- `champ_msgs`: contains info about robot messages
- `champ_teleop`: contains a node to teleoperate the robot using champ

## How to Install

1. Make sure you have ROS2 and Ignition Gazebo (Fortress) downloaded. This version uses ROS2 Jazzy.

2. Make your workspace: 
```
mkdir my_ws
cd my_ws
mkdir src
cd my_ws/src
```

3. Clone the repository into your source folder.
```
git clone [link to this repository]
```

## How to run
1. Source your ROS2 distribution. 
```
source /opt/ros/${ROS_DISTRO}/setup.bash
```

2. Source your workspace.
```
source ~/my_ws/install/setup.bash
```

3. Run the launch file.
```
ros2 launch qped_bringup qped_bringup.launch.py
```
With this, Ignition Gazebo and RViz will both launch.

## How to change worlds/environments
- In the launch file (qped_bringup.launch.py), there are multiple world configurations to change between. To change a world scenario simply change the name of file path in the launch file.

How to add a new environment:
- In qped_gazebo/worlds, add a .sdf file along with the necessary meshes with the correct path directory.
- In the launch file, add a file path to your .sdf and launch it with Ignition Gazebo. See examples in launch file.

## Examples
For different environments, see previous section.

1. Spot Robot in ground plane (simple surface, just a plane)

## Details of Configurations
To see the topics to configure the controllers, run the following commands:

This shows the links/joints of the robot.

```
ros2 run tf2_tools view_frames
```

This shows the nodes/topics to send information to make the robot walk.
```
ros2 run rqt_graph rqt_graph
```
(Working to modify the Twist command with /vox_nav/cmd_vel topic configure controllers and make robot walk in Gazebo)