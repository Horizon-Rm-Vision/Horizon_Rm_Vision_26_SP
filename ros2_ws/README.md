# udp_visualization_bridge

UDP(JSON) → ROS2 bridge (ament_cmake):

- `udp_marker_bridge`: publish `visualization_msgs/msg/MarkerArray` for RViz2.

## Build

```bash
source /opt/ros/humble/setup.bash
cd <your_colcon_ws>
# put this package under src/
colcon build --packages-select udp_visualization_bridge
source install/setup.bash
```
## Run

Marker bridge:
```bash
ros2 run udp_visualization_bridge udp_marker_bridge --ros-args \
  -p port:=9870 \
  -p bind_address:=0.0.0.0 \
  -p marker_topic:=/udp_plot/marker \
  -p frame_id:=map
```

Or launch:
```bash
ros2 launch udp_visualization_bridge udp_visualization_bridge.launch.py
```

## JSON formats

- Key/value object: `{"x": 1.0, "y": 2.0, "z": 0.5}`
- names/values: `{"names":["x","y"],"values":[1.0,2.0]}`
- Array: `[1.0,2.0,3.0]`

For RViz armor cubes (optional): set `target_armor_num` to 3 or 4 and provide `x,y,z,r,l,h`.
