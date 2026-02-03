# StepIt Sim 🤖

**StepIt Sim** is a framework for Sim-to-Sim validation, built upon [stepit](https://github.com/chengruiz/stepit) and [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco). It provides a streamlined workflow for deploying policy networks in Robots.

### 🤖 Supported Robots

* **Unitree G1**
* **Unitree Go2**
* **Unitree B2**
* **Unitree Aliengo**

> *Note: Support for additional robotic platforms will be integrated in future updates.*

---

## 🏗️ Installation & Setup

### 1. Clone the Repository

```bash
git clone https://github.com/legnAray/stepit_sim.git
cd stepit_sim
git submodule update --init --recursive
```

### 2. Install System Dependencies

```bash
sudo apt update && sudo apt install -y cmake build-essential \
  libboost-dev libboost-filesystem-dev libboost-program-options-dev \
  libeigen3-dev libfmt-dev libyaml-cpp-dev \
  libspdlog-dev libglfw3-dev liblcm-dev
```

### 3. Install Unitree SDK2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### 4. Link MuJoCo

Download **MuJoCo 3.3.6** from [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases).

```bash
cd third_party/unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

---

## 🛠️ Build Instructions

### Build unitree_mujoco

```bash
cmake -S third_party/unitree_mujoco/simulate \
      -B third_party/unitree_mujoco/simulate/build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/unitree_mujoco/simulate/build -j
```

### Build StepIt

```bash
export STEPIT_WHITELIST_PLUGINS="control_console;joystick_usb;joystick_udp;csv_publisher;policy_neuro;nnrt_onnxruntime;robot_unitree2"

cmake -S third_party/stepit -B build/stepit -DCMAKE_BUILD_TYPE=Release
cmake --build build/stepit -j
```

---

## 🏃 Running Simulations

### Basic Test

Launch the simulation and use the interactive console:

```bash
./scripts/run_sim.sh -r go2 -s scene.xml -c console -P dummy
```

*Console examples:* `Agent/StandUp`, `Agent/LieDown`

### Policy Simulation

Simulate a learned policy with joystick control:

```bash
./scripts/run_sim.sh -r go2 -s scene.xml -c joystick -f joystick@usb -p path/to/policy_dir -P dummy
```

---

## 🌐 ROS 2 Mode (Optional)

Enable StepIt’s ROS 2 interface for modular control.

### 1. Build ROS 2 Packages

```bash
source /opt/ros/<distro>/setup.zsh
cd ros2_ws
export STEPIT_WHITELIST_PLUGINS="control_console;joystick_usb;joystick_udp;csv_publisher;policy_neuro;nnrt_onnxruntime;robot_unitree2"
colcon build --base-paths src src/stepit/package/ros2 --packages-skip stepit --cmake-args \
  -DSTEPIT_WHITELIST_PLUGINS="$STEPIT_WHITELIST_PLUGINS;ros2_base" \
  -DCMAKE_BUILD_TYPE=Release
```

### 2. Launch in ROS 2 Mode

```bash
source ros2_ws/install/setup.zsh
./scripts/run_sim.sh -ros2 -r go2 -s scene.xml -c ros2_srv -P ros2 -f spin@ros2
```

### 3. Control Commands

| Method | Command |
| --- | --- |
| **Service** | `ros2 service call /control stepit_ros2_msgs/srv/Control "{request: 'Agent/StandUp'}"` |
| **Topic** | `ros2 topic pub --once /control std_msgs/msg/String "{data: 'Agent/StandUp'}"` |


---

### ⚙️ Parameter Configuration

Refer to [stepit](https://github.com/chengruiz/stepit) for additional parameter details.

| Argument | Description | Values |
| --- | --- | --- |
| `-ros2` | Enable ROS 2 interface | (Flag) |
| `-r` | Robot model | `aliengo`, `go2`, `b2`, `g1` |
| `-s` | Scene XML file | `scene.xml` |
| `-c` | Controller type | `console`, `joystick`, `ros2_msg`, `ros2_srv`, `dummy` |
| `-p` | Policy directory path | `/path/to/policy_dir` |
| `-P` | Publisher type | `dummy`, `csv`, `ros2` |
| `-f` | default factory for a specified type | `joystick@usb`, `spin@ros2`, `spin@wait_for_sigint` |
| `-v` | Log level | `0(Error)`, `1(Warn)`, `2(Info)`, `3(Debug)` |
| `--` | **Pass-through** | Forward CLI args to StepIt plugins (e.g., ROS 2 args) |

#### 💡 Pass-through Examples

Use `--` to pass arguments directly to StepIt plugins. This is particularly useful for ROS 2 node remapping or namespacing:

```bash
# Example: Passing ROS 2 arguments to set a namespace
./scripts/run_sim.sh -ros2 -r go2 -s scene.xml -c joystick -f joystick@usb -p path/to/policy_dir -P ros2 -- --ros-args -r __ns:=/go2
```

---

## 🤝 Acknowledgements

This repository is built upon the following open-source projects:

* [stepit](https://github.com/chengruiz/stepit)
* [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
