# assignment2_rt1

A ROS 2 package: a robot (or simulator) is driven by user-supplied velocity commands while a laser-based safety layer stops and reverses it whenever it gets too close to an obstacle.

![ROS2 RT1 Assignment2 overview](image/assignment2_rt1.svg)

## Nodes

- **ui_node** — reads `linear angular` pairs from the keyboard and publishes them as a `Twist` on `/user_request`.
- **laser_status** — subscribes to `/scan`, finds the closest obstacle and its direction, and publishes a custom `RobotStatus` message (`distance`, `direction`, `threshold`) on `/robot_status`. It also exposes the `set_safety_threshold` service to change the safety distance at runtime.
- **controller** — subscribes to `/user_request` and `/robot_status`. If the robot is within the safety threshold it stops and reverses the last command instead of applying the new one; otherwise it forwards the requested velocity as a `Twist` on `/cmd_vel`. It keeps the last 5 user commands and exposes the `get_avg_service` service, which returns their average linear/angular velocity.

## How to run

Clone this package into the `src` folder of a ROS 2 workspace and build it:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Tawakoll/assignment2_rt1.git
cd ~/ros2_ws
colcon build --packages-select assignment2_rt1
source install/setup.bash
```

Then, in separate terminals (each sourced with `source install/setup.bash`):

```bash
# 1. Start the robot/simulator (must publish /scan and subscribe to /cmd_vel),
#    e.g. a Stage/Gazebo simulation launched with your preferred launch file

# 2. Start the laser status node
ros2 run assignment2_rt1 laser_status.py

# 3. Start the controller node
ros2 run assignment2_rt1 controller.py

# 4. Start the user interface and follow the prompts
ros2 run assignment2_rt1 ui_node.py
```

In the `ui_node` terminal, enter a linear and angular velocity separated by a space (e.g. `0.5 0.0`), or `x` to exit.

### Calling the services

```bash
# Change the safety threshold (in meters)
ros2 service call /set_safety_threshold assignment2_rt1/srv/SetThreshold "{new_threshold: 1.5}"

# Get the average of the last 5 user commands
ros2 service call /get_avg_service assignment2_rt1/srv/GetAvg
```
