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
sudo apt update && sudo apt install -y \
  ca-certificates curl git build-essential \
  libboost-dev libboost-filesystem-dev libboost-program-options-dev \
  libeigen3-dev libfmt-dev libyaml-cpp-dev \
  libspdlog-dev libglfw3-dev liblcm-dev
```

### 3. Prepare Workspace-local CMake

StepIt requires CMake 3.23 or newer. Ubuntu 22.04 ships CMake 3.22.1, so this repository uses a workspace-local CMake under `tools/cmake` instead of changing the system CMake.

```bash
./scripts/setup_cmake.sh
export PATH="$PWD/tools/cmake/bin:$PATH"
cmake --version
```

Run the `export PATH=...` line again in each new terminal before building. Do not run `third_party/stepit/scripts/setup.sh` for this repository; that script is for standalone StepIt workspaces, while `stepit_sim` uses the pinned `third_party/stepit` submodule.

### 4. Install Unitree SDK2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

### 5. Link MuJoCo

Download **MuJoCo 3.3.6** from [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases).

```bash
cd third_party/unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

---

## 🛠️ Build Instructions

There are two build profiles:

- **Basic simulation:** MuJoCo + Unitree DDS control, no ROS 2 perception topics.
- **ROS 2 perception:** MuJoCo publishes raycaster/GridMap topics, and StepIt can use `policy_neuro_ros2::field_subscriber` / `heightmap_subscriber`.

If you opened a new terminal after setup, enable the workspace-local CMake first:

```bash
export PATH="$PWD/tools/cmake/bin:$PATH"
```

### Basic Simulation Build

Build `unitree_mujoco` without ROS 2 publishers:

```bash
cmake -S third_party/unitree_mujoco/simulate \
      -B third_party/unitree_mujoco/simulate/build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/unitree_mujoco/simulate/build -j
```

Build StepIt without ROS 2 plugins:

```bash
export STEPIT_WHITELIST_PLUGINS="control_console;joystick_usb;joystick_udp;publisher_csv;policy_neuro;nnrt_onnxruntime;robot_unitree2"

cmake -S third_party/stepit -B build/stepit \
  -DSTEPIT_WHITELIST_PLUGINS="$STEPIT_WHITELIST_PLUGINS" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/stepit -j
```

Run with the normal StepIt binary:

```bash
./scripts/run_sim.sh -r go2 -s scene.xml -c joystick -f joystick@usb -p path/to/policy_dir -P dummy
```

### ROS 2 Perception Build

Install the extra ROS 2 packages needed by the simulator perception publishers:

```bash
sudo apt install -y \
  ros-<distro>-grid-map-msgs ros-<distro>-grid-map-ros ros-<distro>-grid-map-cv \
  ros-<distro>-pcl-conversions ros-<distro>-tf2-ros
```

Build `unitree_mujoco` with ROS 2 raycaster/GridMap publishers:

```bash
source /opt/ros/<distro>/setup.zsh

cmake -S third_party/unitree_mujoco/simulate \
      -B third_party/unitree_mujoco/simulate/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_ROS2=ON
cmake --build third_party/unitree_mujoco/simulate/build -j
```

Build StepIt in the bundled ROS 2 workspace with `policy_neuro_ros2` enabled:

```bash
source /opt/ros/<distro>/setup.zsh
cd ros2_ws

export STEPIT_WHITELIST_PLUGINS="control_console;joystick_usb;joystick_udp;publisher_csv;policy_neuro;policy_neuro_ros2;nnrt_onnxruntime;robot_unitree2"
colcon build --base-paths src src/stepit/package/ros2 --packages-skip stepit --cmake-args \
  -DSTEPIT_WHITELIST_PLUGINS="$STEPIT_WHITELIST_PLUGINS;ros2_base" \
  -DCMAKE_BUILD_TYPE=Release
```

Run with the ROS 2 StepIt binary:

```bash
source /opt/ros/<distro>/setup.zsh
source ros2_ws/install/setup.zsh
./scripts/run_sim.sh -ros2 -r go2 -s scene.xml -c ros2_srv -P ros2 -f spin@ros2
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

### Template

Use the following template to launch the `g1` simulation with the bundled example policy:

```bash
./scripts/run_sim.sh -r g1 -s scene.xml -c joystick -f joystick@usb -p ./template_policy/g1_amp -P dummy
```

> **Note for `g1`:** The simulation includes a virtual elastic band for lifting and lowering the robot. Use keyboard `7` to lower the robot, `8` to lift it, and `9` to enable or release the suspension.

---

## 🌐 ROS 2 Mode (Optional)

Enable StepIt’s ROS 2 interface for modular control.

To feed terrain/depth into policies, keep `policy_neuro_ros2` in the whitelist. The simulator can publish `/height_scan_array` and `/depth_camera_array` for `field_subscriber`; `height_scan` defaults to `output_format: "both"`, so `/height_scan` is also available for `/elevation_map` and `heightmap_subscriber` when `enable_gridmap: true`.
Raycaster sensors are defined in the MuJoCo XML but default to disabled; enable them explicitly with `--heightmap` and/or `--depth`.

### Launch in ROS 2 Mode

```bash
source /opt/ros/<distro>/setup.zsh
source ros2_ws/install/setup.zsh
./scripts/run_sim.sh -ros2 -r go2 -s scene.xml -c ros2_srv -P ros2 -f spin@ros2
```

### Control Commands

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
| `--heightmap` / `--height-scan` | Enable MuJoCo `height_scan` raycaster | (Flag) |
| `--depth` / `--depth-camera` | Enable MuJoCo `depth_camera` raycaster | (Flag) |
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
