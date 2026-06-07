# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

## Native on Ubuntu 20.04

**Install the following dependencies:**
- **ROS 2** Follow the instructions [here](https://docs.ros.org/en/foxy/Installation.html) to install ROS 2 Foxy.
- **F1TENTH Gym**
  ```bash
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
  ```
- Update correct parameter for path to map file:
  Go to `sim.yaml` [https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml](https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml) in your cloned repo, change the `map_path` parameter to point to the correct location. It should be `'<your_home_dir>/sim_ws/src/f1tenth_gym_ros/maps/levine'`
- Install dependencies with rosdep:
  ```bash
  source /opt/ros/foxy/setup.bash
  cd ..
  rosdep install -i --from-path src --rosdistro foxy -y
  ```
- Build the workspace: ```colcon build```

## With an NVIDIA gpu:

**Install the following dependencies:**

- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker. If you followed the post-installation steps you won't have to prepend your docker and docker-compose commands with sudo.
- **nvidia-docker2**, follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) if you have a support GPU. It is also possible to use Intel integrated graphics to forward the display, see details instructions from the Rocker repo. If you are on windows with an NVIDIA GPU, you'll have to use WSL (Windows Subsystem for Linux). Please refer to the guide [here](https://developer.nvidia.com/cuda/wsl), [here](https://docs.nvidia.com/cuda/wsl-user-guide/index.html), and [here](https://dilililabs.com/zh/blog/2021/01/26/deploying-docker-with-gpu-support-on-windows-subsystem-for-linux/).
- **rocker** [https://github.com/osrf/rocker](https://github.com/osrf/rocker). This is a tool developed by OSRF to run Docker images with local support injected. We use it for GUI forwarding. If you're on Windows, WSL should also support this.

**Installing the simulation:**

1. Clone this repo
2. Build the docker image by:
```bash
$ cd sim_ws/src/f1tenth_gym_ros
$ docker build -t f1tenth_gym_ros -f Dockerfile .
```
3. To run the containerized environment, start a docker container by running the following. (example showned here with nvidia-docker support). By running this, the current directory that you're in (should be `f1tenth_gym_ros`) is mounted in the container at `/sim_ws/src/f1tenth_gym_ros`. Which means that the changes you make in the repo on the host system will also reflect in the container.
```bash
$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

# Launching the Simulation

- Initial workspace: `source ./install/setup.bash`
- Start simulator: `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
- Start teleop (joy, teleop & mux): `ros2 launch f1tenth_gym_ros bringup_lanuch.py`

**Algorithm**
- AEB: `ros2 run safety_node safety_node_cpp`
- Wall following: `ros2 run wall_follow wall_follow_node_cpp`
- Follow the gap: `ros2 run gap_follow reactive_node_cpp`
- Waypoint generator: `ros2 run waypoint_generator waypoint_generator`
- Pure pursuit: `ros2 launch pure_pursuit sim_pure_pursuit_launch.py`

**Option**
- Dynamic Reconfigure: `rqt -s rqt_reconfigure`
