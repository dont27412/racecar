# turtlebot3_wp_following

A waypoint following package for TurtleBot3 that is built **on top of Nav2**.

Instead of publishing `cmd_vel` directly, this node sends each waypoint to Nav2
through the `NavigateToPose` action. Nav2 then plans the route with the global
planner and publishes it on the `/plan` (global) and `/local_plan` topics. Those
topics are already configured as Path displays in the stock TurtleBot3 RViz
configuration, so **the planned path appears on the map in RViz while the robot
drives** from waypoint to waypoint.

## Build

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_wp_following
source install/setup.bash
```

## Run

1. Bring up Nav2 with your own map:

   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
       use_sim_time:=true map:=/home/mlh090300/my_map.yaml
   ```

2. In RViz, use **2D Pose Estimate** to set the robot's initial pose on the map
   (only needed once, so AMCL can localize).

3. Start the waypoint follower:

   ```bash
   ros2 launch turtlebot3_wp_following wp_following.launch.py
   ```

The follower waits for the Nav2 action server, then sequentially navigates
through the waypoints in `config/wp_following.yaml`. The path shows on RViz.

## Waypoints

Edit `config/wp_following.yaml`:

```yaml
waypoints:
  - {x: 1.5, y: 0.0, yaw: 0.0}
  - {x: 1.5, y: 1.0, yaw: 1.57}
```

Each entry is a pose in the `map` frame: `x`, `y` (metres) and `yaw` (radians).

Launch arguments:

| Argument         | Default            | Description                                  |
|------------------|--------------------|----------------------------------------------|
| `use_sim_time`   | `true`             | Use the Gazebo simulation clock              |
| `waypoints_file` | packaged yaml      | Full path to a waypoints YAML                |
| `loop`           | `false`            | Loop back to the first waypoint when done    |
| `goal_delay`     | `2.0`              | Seconds to wait after each goal completes    |
