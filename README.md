# stepit_sim
Sim-to-sim validation with [stepit](https://github.com/legnAray/stepit) and [unitree_mujoco](https://github.com/legnAray/unitree_mujoco).


## Quick Start

### 1) Clone

```bash
git clone https://github.com/legnAray/stepit_sim.git
cd stepit_sim
git submodule update --init --recursive
```

### 2) System dependencies

```bash
sudo apt install cmake build-essential \
  libboost-dev libboost-filesystem-dev libboost-program-options-dev \
  libeigen3-dev libfmt-dev libyaml-cpp-dev \
  libspdlog-dev libglfw3-dev
```

### 3) Install unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### 4) Link MuJoCo

Download MuJoCo 3.3.6 from: https://github.com/google-deepmind/mujoco/releases

```bash
cd third_party/unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

### 5) Build unitree_mujoco

```bash
cmake -S third_party/unitree_mujoco/simulate \
      -B third_party/unitree_mujoco/simulate/build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/unitree_mujoco/simulate/build -j
```

Run the simulator (Go2 example):

```bash
./third_party/unitree_mujoco/simulate/build/unitree_mujoco -r go2 -s scene.xml -i 1 -n lo
```

### 6) Build stepit

Default plugin whitelist in this repo:
`control_console;joystick_usb;joystick_udp;csv_publisher;policy_neuro;nnrt_onnxruntime;robot_unitree2;robot_unitree_go1;robot_unitree_b1;robot_unitree_aliengo`

```bash
cmake -S third_party/stepit -B build/stepit -DCMAKE_BUILD_TYPE=Release
cmake --build build/stepit -j
```

### 7) Run stepit (console control)

```bash
export STEPIT_NETIF=lo
export STEPIT_DOMAIN_ID=1
./build/stepit/bin/stepit -r go2 -c console -P dummy
```

Console input examples:

```
Agent/StandUp
Agent/LieDown
```

## ROS2 mode (optional)

This enables StepIt’s ROS2 interface.

### 1) Build ROS2 packages

```bash
source /opt/ros/<distro>/setup.zsh
cd ros2_ws
colcon build --cmake-args \
  -DSTEPIT_WHITELIST_PLUGINS="control_console;joystick_usb;joystick_udp;csv_publisher;policy_neuro;nnrt_onnxruntime;robot_unitree2;robot_unitree_go1;robot_unitree_b1;robot_unitree_aliengo;ros2_base" \
  -DCMAKE_BUILD_TYPE=Release
```

### 2) Start unitree_mujoco

```bash
./third_party/unitree_mujoco/simulate/build/unitree_mujoco -r go2 -s scene.xml -i 1 -n lo
```

### 3) Start stepit (ROS2)

```bash
source /opt/ros/<distro>/setup.zsh
source ros2_ws/install/setup.zsh
export STEPIT_NETIF=lo
export STEPIT_DOMAIN_ID=1

ros2_ws/install/stepit_ros2/lib/stepit_ros2/stepit -r go2 -c ros2_srv -P ros2 -f spin@ros2
```

### 4) Send ROS2 control commands

Service:

```bash
ros2 service call /control stepit_ros2_msgs/srv/Control "{request: 'Agent/StandUp'}"
ros2 service call /control stepit_ros2_msgs/srv/Control "{request: 'Agent/LieDown'}"
```

Topic:

```bash
ros2 topic pub --once /control std_msgs/msg/String "{data: 'Agent/StandUp'}"
```

## Acknowledgements

This repository is built on top of:

- stepit: https://github.com/chengruiz/stepit
- unitree_mujoco: https://github.com/unitreerobotics/unitree_mujoco