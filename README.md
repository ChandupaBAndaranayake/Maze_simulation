# Maze Simulation ROS 2 Jazzy Setup

## 🐳 Docker Container

You are using a Docker container based on `osrf/ros:jazzy-desktop-full`. This contains ROS 2 Jazzy full desktop tools, Gazebo, and RViz.

Your Dockerfile also sets up:

* TurtleBot3 environment (`TURTLEBOT3_MODEL=burger`)
* User `ros` with sudo privileges
* X11 forwarding for GUI applications
* Workspace mount: `./ros_ws:/home/ros/ros_ws`

---

## ⚡ Required ROS 2 Packages

To run the provided launch files for **SLAM** and **Nav2**, make sure the following packages are installed inside the container.

### 1. TurtleBot3 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-bringup \
  ros-jazzy-turtlebot3-navigation2
```

* **Purpose**:

  * `turtlebot3-msgs`: Robot messages.
  * `turtlebot3-gazebo`: Gazebo simulation for TurtleBot3.
  * `turtlebot3-bringup`: State publisher & basic robot bringup.
  * `turtlebot3-navigation2`: Nav2 bringup for TurtleBot3.

---

### 2. SLAM Toolbox

```bash
sudo apt install -y ros-jazzy-slam-toolbox
```

* **Purpose**: Provides online mapping and localization nodes used in `slam_toolbox` launch files.

---

### 3. Navigation 2 Stack

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-route

```

* **Purpose**: Provides AMCL localization, planners, controllers, and BT navigator for autonomous navigation.

---

### 4. Additional Tools

These are already included in your Dockerfile but make sure they are installed:

```bash
sudo apt install -y \
  rviz2 \
  twist-mux \
  x11-apps \
  net-tools \
  iputils-ping \
  git \
  curl
```

* `twist-mux` is required for velocity control remapping (`cmd_vel`).
* `rviz2` for visualization of SLAM maps and navigation.
* X11 apps and `LIBGL_ALWAYS_INDIRECT=1` allow GUI apps like RViz and Gazebo to display.

---

## 📁 ROS Workspace

Your container mounts a workspace at `/home/ros/ros_ws`. Clone your Maze Simulation repository inside it:

```bash
cd ~/ros_ws/src
git clone https://github.com/ChandupaBAndaranayake/Maze_simulation.git
```

Build the workspace:

```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🛠 Launching

### 1. SLAM

Use `slam.launch.py` (from your third launch file):

```bash
ros2 launch maze_simulation slam.launch.py
```

* Spawns TurtleBot3 in Gazebo.
* Runs the **SLAM Toolbox** (`online_async`).
* Starts RViz with your SLAM config (`slam_config.rviz`).

---

### 2. Navigation 2

Use `navigation.launch.py` (from your second launch file):

```bash
ros2 launch maze_simulation navigation.launch.py
```

* Spawns TurtleBot3 in Gazebo.
* Starts **Nav2 stack**: AMCL, map server, planners, controllers.
* Launches **twist_mux** for command arbitration.
* Opens RViz with your Nav2 configuration (`tb3_navigation2.rviz`).

---

## 📝 Notes

1. **Gazebo Worlds & Models**:

   * Maze world: `maze_world.sdf` (in `worlds/` folder).
   * Robot spawn: uses `TURTLEBOT3_MODEL=burger`.
   * Gazebo models paths are added to `GZ_SIM_RESOURCE_PATH`.

2. **Delays in Launch Files**:

   * Delayed launches (`TimerAction`) ensure Gazebo is fully loaded before spawning robot, SLAM, or Nav2 nodes.

3. **Environment Variables**:

   * `DISPLAY=host.docker.internal:0.0` for X11 forwarding.
   * `LIBGL_ALWAYS_INDIRECT=1` for OpenGL rendering in Docker.
