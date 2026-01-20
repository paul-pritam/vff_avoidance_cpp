# VFF Avoidance

A ROS 2 package implementing Vector Field Force (VFF) obstacle avoidance for mobile robots. This package uses laser scan data to generate attractive and repulsive forces, combining them to produce smooth navigation commands that both move the robot toward a goal and avoid detected obstacles.

## Overview

The `vff_avoidance` package implements the Vector Field Force algorithm, a potential field method where:

- **Attractive Forces**: Pull the robot toward a goal (forward direction)
- **Repulsive Forces**: Push the robot away from detected obstacles
- **Resulting Vector**: Combination of both forces guides the robot's motion

This approach provides smooth, reactive obstacle avoidance without requiring explicit path planning.

## Features

- **Real-time Obstacle Avoidance**: Reactive navigation based on laser scan data
- **Vector Field Force Algorithm**: Combines attractive and repulsive forces
- **Debug Visualization**: RViz markers display force vectors for debugging
- **Configurable Parameters**: Easily adjust obstacle distance threshold and force magnitudes
- **Efficient Control Loop**: Fixed-rate 50ms control cycle
- **ROS 2 Native**: Built with modern ROS 2 architecture

## Dependencies

- **ROS 2**: Core middleware
- **rclcpp**: ROS 2 C++ client library
- **geometry_msgs**: Robot motion commands
- **sensor_msgs**: Laser scan data
- **visualization_msgs**: Debug visualization markers

## Architecture

### Main Component

#### VffAvoidance Node

- Subscribes to laser scan messages (`input_scan`)
- Computes attractive and repulsive force vectors using VFF algorithm
- Publishes velocity commands (`output_vel`) as `Twist` messages
- Publishes debug markers (`vff_debug`) for visualization in RViz
- Runs at 50ms control cycle

### Algorithm Details

**Vector Computation:**

1. **Attractive Force**: Constant forward vector (1.0, 0.0) pulling toward goal
2. **Repulsive Force**: Based on nearest obstacle
   - Calculated only if obstacle is within `OBSTACLE_DIST` (1.5m)
   - Magnitude increases as obstacle gets closer: `(OBSTACLE_DIST - min_dist) / OBSTACLE_DIST * 5`
   - Direction opposite to obstacle: `-cos(angle) * magnitude, -sin(angle) * magnitude`
3. **Result Vector**: Sum of attractive and repulsive forces
4. **Velocity Output**:
   - Linear velocity: Magnitude of result vector (clamped to 0.3 m/s)
   - Angular velocity: Angle of result vector (clamped to ±0.5 rad/s)

## Building

From the workspace root:

```bash
colcon build --packages-select vff_avoidance
```

## Running

### Launch the Node

```bash
ros2 run vff_avoidance avoidance_vff_node
```

### With Visualization

```bash
# Terminal 1: Start simulation with laser
ros2 launch mybot launch_sim.launch.py

# Terminal 2: Run avoidance node
ros2 run vff_avoidance avoidance_vff_node

# Terminal 3: View in RViz
ros2 launch mybot rsp.launch.py

# In RViz, add visualization for /vff_debug (Marker Array) to see force vectors
```

## Topics

### Subscribed Topics

- `input_scan` (`sensor_msgs/LaserScan`): Laser scan data from robot sensors

### Published Topics

- `output_vel` (`geometry_msgs/Twist`): Velocity commands for robot motion
  - `linear.x`: Forward velocity (0.0-0.3 m/s)
  - `angular.z`: Rotational velocity (±0.5 rad/s)
- `vff_debug` (`visualization_msgs/MarkerArray`): Debug visualization of force vectors
  - Red marker: Repulsive force
  - Green marker: Attractive force
  - Blue marker: Resultant force

## Configuration

Key parameters in [avoidance_node.cpp](src/vff_avoidance/avoidance_node.cpp):

- **OBSTACLE_DIST**: Detection radius for obstacle avoidance (1.5m)
- **Repulsive Magnitude Factor**: `5.0` - adjusts strength of obstacle repulsion
- **Linear Velocity Limit**: `0.3` m/s - maximum forward speed
- **Angular Velocity Limit**: `0.5` rad/s - maximum rotation rate
- **Control Cycle**: `50ms` - frequency of updates

### Tuning Tips

- Increase `OBSTACLE_DIST` for earlier obstacle detection
- Adjust repulsive magnitude factor for stronger/weaker obstacle response
- Modify velocity limits for faster/slower robot motion
- Reduce control cycle (increase frequency) for smoother response

## Testing

### Unit Tests

```bash
colcon test --packages-select vff_avoidance
```

View test results:

```bash
colcon test --packages-select vff_avoidance --verbose
```

### Integration Testing with Simulation

1. Launch simulation with obstacles
2. Run avoidance node
3. Monitor topics:
   ```bash
   ros2 topic echo /output_vel
   ros2 topic echo /input_scan
   ```
4. Visualize in RViz:
   - Add TF to see robot orientation
   - Add Marker Array for `/vff_debug` to see force vectors
   - Add LaserScan for `/input_scan` to see sensor data

## Node Graph

```
/input_scan (LaserScan)
    |
    v
[VffAvoidance Node]
    |
    +---> /output_vel (Twist) ---> Robot Motion Controller
    |
    +---> /vff_debug (MarkerArray) ---> RViz Visualization
```

## How VFF Works - Visual Example

```
Obstacle Detected
        |
        v
    (O) <- Repulsive Force (away from obstacle)
   /
  /
Robot ---> Attractive Force (toward goal)
  \
   \
    \ Resultant = Attractive + Repulsive
     v
   [Navigation Direction]
```

When obstacles are far: Attractive force dominates → move toward goal
When obstacles approach: Repulsive force grows → avoid obstacle
Balance achieved: Smooth arc around obstacle

## Troubleshooting

### Robot Not Moving
- Verify laser scan is being published: `ros2 topic echo /input_scan`
- Check if scan data contains valid ranges (not all NaN/Inf)
- Ensure output velocity is being subscribed by the robot controller

### Oscillating or Erratic Behavior
- Reduce linear velocity limit for smoother motion
- Decrease repulsive magnitude factor for gentler obstacle response
- Increase control cycle frequency for more responsive updates

### No Obstacle Avoidance
- Verify `OBSTACLE_DIST` threshold is appropriate for your environment
- Check laser scan range and angle parameters
- Ensure obstacles are within sensor range

## Performance

- **Control Cycle**: 50ms (20 Hz update rate)
- **Computation Time**: < 1ms per cycle
- **Memory Usage**: Minimal, efficient vector operations
- **CPU Load**: Low single-core usage

## Future Enhancements

- Dynamic goal attraction based on external input
- Multiple goal waypoint support
- Tunable parameters via ROS parameters
- Advanced collision prediction
- Integration with global path planning

## License

Apache License 2.0 - See LICENSE file for details

## Maintainer

- **Email**: pritampaulwork7@gmail.com
- **Author**: ubuntu
