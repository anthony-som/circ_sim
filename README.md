# circ_sim — Mini Arm + Table Simulation (ROS 2 Jazzy)

Gazebo Harmonic simulation of a 6-DOF mini arm (with an end-effector camera)
mounted at a table with buttons, cubes and a transport box.

Tested on **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**.

---

## 1. Install ROS 2 Jazzy

Skip if `/opt/ros/jazzy` already exists.

```bash
sudo apt update && sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-pip
```

## 2. Install project dependencies

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-gz-ros2-control-demos \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
  ros-jazzy-joy ros-jazzy-rviz2 ros-jazzy-rqt-image-view python3-numpy
pip install --break-system-packages ikpy
```

## 3. Clone and build

```bash
cd ~
git clone -b jazzy https://github.com/anthony-som/circ_sim.git
cd ~/circ_sim

source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

Optional — source automatically in every new terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/circ_sim/install/setup.bash" >> ~/.bashrc
```

---

## 4. Launch (3 terminals)

Every terminal needs the environment sourced first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/circ_sim/install/setup.bash
```

### Terminal 1 — World

Starts Gazebo with the table world, button contact bridges and the button monitor.

```bash
ros2 launch table world.launch.py
```

Wait until the Gazebo window is open and the table is visible.

### Terminal 2 — Arm

Spawns the arm into the running world, starts the controllers, the camera
bridge, RViz and the arm camera feed window.

```bash
ros2 launch mini_arm_ros2 gazebo.launch.py launch_gazebo:=false world:=empty
```

This is staged on timers: arm spawns at ~3 s, controllers at ~15 s,
RViz + camera feed at ~22 s. Optional spawn position: `x:=-2.8 y:=-0.656 z:=1.12`.

### Terminal 3 — Teleop

Start once the controllers are active (after ~15 s in terminal 2).

```bash
ros2 run mini_arm_teleop keyboard_teleop
```

Keep this terminal focused while pressing keys (no Enter needed):

| Joint            | + / − |
|------------------|-------|
| 1 base           | q / a |
| 2 shoulder       | w / s |
| 3 elbow          | e / d |
| 4 wrist          | r / f |
| 5 end            | t / g |
| 6 gripper        | y / h |
| Step size        | [ / ] |
| Reset all        | 0     |
| Quit             | x     |

Other teleop options:

```bash
ros2 run mini_arm_teleop keyboard_cartesian_ik_teleop
ros2 run mini_arm_teleop ps5_arm_teleop             # also run: ros2 run joy joy_node
ros2 run mini_arm_teleop ps5_cartesian_ik_teleop    # also run: ros2 run joy joy_node
ros2 run mini_arm_teleop logitech3d_arm_teleop      # also run: ros2 run joy joy_node
```

---

## Arm camera feed

The camera on the end effector publishes `/arm_camera/image_raw`.
It opens automatically (rqt_image_view window + "ArmCamera" panel in RViz).
To open it manually:

```bash
ros2 run rqt_image_view rqt_image_view /arm_camera/image_raw
```

## Other launch modes

```bash
# Arm alone in an empty Gazebo world
ros2 launch mini_arm_ros2 gazebo.launch.py

# Arm in RViz only with mock hardware (no Gazebo)
ros2 launch mini_arm_ros2 bringup.launch.py
```

## Troubleshooting

```bash
ros2 control list_controllers              # both controllers should be "active"
ros2 topic hz /arm_camera/image_raw        # camera frame rate
gz topic -l | grep arm_camera              # camera exists on the Gazebo side
```

- **Camera feed blank in RViz:** use the *Image* display (not *Camera*) and set
  Reliability Policy to *Best Effort*.
- **Teleop keys do nothing:** the teleop terminal must have keyboard focus, and
  the controllers must be active.
- **Low camera FPS on VMs:** expected with software rendering.
