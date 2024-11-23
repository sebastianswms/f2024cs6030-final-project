Once:

```bash
cd ~/ssmiley/f2024cs6030-final-project/project/snowplow_ws &&
cp ~/Downloads/yolov3.weights ./camera
```

```bash
cd ~/ssmiley/f2024cs6030-final-project/project/snowplow_ws &&
colcon build &&
cd /opt/carla-simulator/ &&
./CarlaUE4.sh
```

Every Time:

```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

```bash
ros2 service call /carla/spawn_object carla_msgs/srv/SpawnObject "{type: 'walker.pedestrian.0001', id: 'pedestrian_1', transform: {position: {x: 62.0, y: 85.0, z: 8.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}"
```

```bash
cd ~/ssmiley/f2024cs6030-final-project/project/snowplow_ws &&
python3 camera.py
```

```bash
cd ~/ssmiley/f2024cs6030-final-project/project/snowplow_ws &&
python3 lidar_radar.py
```

```bash
cd ~/ssmiley/f2024cs6030-final-project/project/snowplow_ws &&
python3 ppc2.py
```
