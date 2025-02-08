# ros2-course
DevContainer setup for ROS2 course


## Prerequisites

### Linux
Install the following:
- Docker Engine
- VS Code
- VS Code extension: Dev Containers

Follow the listed steps to finish setup:
1. Create a json file in the `.devcontainer` dir named `devcontainer.json` and copy the content of `devcontainer - linux.json` into it. 

### Windows
Install the following:
- Docker Engine
- VS Code
- VS Code extension: Dev Containers
- XLaunch or other Xserver

Follow the listed steps to finish setup:
1. Create a json file in the `.devcontainer` dir named `devcontainer.json` and copy the content of `devcontainer - windows.json` into it. 
2. Start a Xserver with 0.0 as DISPLAY port

## Start DevContainer
1. Open this repository in VS Code.
2. `crtl + shift + P` to open command prompt
3. Enter `Dev Containers: Rebuild and Reopen in Container` 

VS Code will start the Dev Container building the Dockerfile found in the `/docker` dir. This is a basic ROS2 Jazzy Container with the following ROS related binaries installed:
- `ros-jazzy-plotjuggler-ros`, a topic plotter package.
- `ros-jazzy-moveit`, binary install of the base MoveIt2 packages
- `ros-jazzy-ros-gz`, binary install of gz-sim11 Gazebo Harmonic ROS2
- `'~nros-jazzy-rqt*'`, RQT for ROS2 Jazzy

## Installing in Container
The `~/ros2_ws/src` directory in the container is bound to the `src` directory in this repository. Which means source installs are persistent and saved when stopping and/or rebuilding the DevContainer.
The opposite is true for binary installs, these are not persistent. Unless added to the Dockerfile. 

ROS2 packages should be installed/cloned into the `~/ros2_ws/src` folder. 
To install the dependencies for these packages use:
``` bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y -r
```
This should install all package dependencies referenced in the `package.xml` files.
To build the workspace use:
``` bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Start BFFP Demo Factory
After installing the rosdeps and building and sourcing the workspace (see [Installing in Container](#installing-in-container)), use: `ros2 launch bffp_gazebo factory_moveit_full.launch.py` to launch the demo. This launches a Gazebo simulation and a HC10DTP with full MoveIt2 support. Rviz2 is also launched to visualize ROS2 topics.

## Start Tiago simulation
### Tiago 2D navigation using Nav2
**Documentation** : https://docs.pal-robotics.com/sdk-dev/navigation
1️⃣ Start Gazebo simulation of Tiago robot:
``` bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house_pick_and_place
```
2️⃣ In a new terminal, start cartographer to run SLAM:
``` bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py slam:=True is_public_sim:=True
```
3️⃣ In a new terminal, start teleoperation from the keyboard:
``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/key_vel
```
Then move the robot to accumulate map data.
Close teleoperation with Ctrl+C and save map to file `our_map` using:
``` bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/pal_maps/maps/our_map/map
```
Close the SLAM terminal with Ctrl+C.
If you have not used `--symlink-install`, workspace building and sourcing are needed so that the new map is installed.
4️⃣ Start 2D navigation by loading your `our_map` map, using:
``` bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map
```
⚠️ Teleoperation and navigation both publish Twist robot commands to `/cmd_vel`: make sure they don't publish at the same time, your robot would receive contradictory commands.
5️⃣ In RViz use the arrows to control navigation:
1. **Estimate 2D pose** to initialize random particles to an approximate position around the actual robot position
2. Send a **2D nav goal** to any point of the map: your robot must navigate there
6️⃣ Command navigation from Python code using the `navigate_to_pose` action service (see workshops instructions)
7️⃣ **4-waypoints robot patrol**: Inspire from the `goto` code to add a new patrol node, endlessly patroling between 4 map poses.  

### Tiago arms and gripper manipulation using MoveIt2
**Documentation**: https://docs.pal-robotics.com/sdk-dev/manipulation
8️⃣ Command MoveIt using color handles from the GUI in RViz:
```bash
ros2 launch tiago_moveit_config moveit_rviz.launch.py
```
9️⃣ Command MoveIt from Python file `ros2_ws/src/tiago_pick_and_place/tiago_pick_and_place/pick.py`:

```bash
ros2 launch tiago_pick_and_place plan.launch.py use_sim_time:=True
```
