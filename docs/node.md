# Car/Simulator

## styx_server

### subscriber
- [`/vehicle/steering_cmd[dbw_mkz_msgs/SteeringCmd]`](./msg.md#steering_cmd)
- [`/vehicle/throttle_cmd[dbw_mkz_msgs/ThrottleCmd]`](./msg.md#throttle_cmd)
- [`/vehicle/brake_cmd[dbw_mkz_msgs/BrakeCmd]`](./msg.md#brake_cmd)

### publisher
- [`/current_pose[geometry_msgs/PoseStamped]`](./msg.md#current_pose)
- [`/current_velocity[geometry_msgs/TwistStamped]`](./msg.md#current_velocity)
- [`/vehicle/steering_report[dbw_mkz_msgs/SteeringReport]`](./msg.md#steering_report)
- [`/vehicle/throttle_report[std_msgs/Float32]`](./msg.md#throttle_report)
- [`/vehicle/brake_report[std_msgs/Float32]`](./msg.md#brake_report)
- [`/vehicle/obstacle[geometry_msgs/PoseStamped]`](./msg.md#obstacle)
- [`/vehicle/obstacle_points[sensor_msgs/PointCloud2]`](./msg.md#obstacle_points)
- [`/vehicle/lidar[sensor_msgs/PointCloud2]`](./msg.md#lidar)
- [`/vehicle/traffic_lights[styx_msgs/TrafficLightArray]`](./msg.md#traffic_lights)
- [`/vehicle/dbw_enabled[std_msgs/Bool]`](./msg.md#dbw_enabled)
- [`/image_color[sensor_msgs/Image]`](./msg.md#image_color)

## camera_info_publisher

### subscriber 
- None

### publisher
- [camera_info]

# Perception

## Traffic Light Detection Node[tl_detctor]

### subscriber 
- [`/current_pose[geometry_sensor_msgs/Imagemsgs/PoseStamped]`](./msg.md#current_pose)
- [`/base_waypoints[styx_msgs/Lane]`](./msg.md#base_waypoints)
- [`/image_color[sensor_msgs/Image]`](./msg.md#image_color)
- **SIM ONLY** [`/vehicle/traffic_lights[styx_msgs/TrafficLightArray]`](./msg.md#traffic_lights)

### publisher
- [`/traffic_waypoint[std_msgs/Int32]`](./msg.md#traffic_waypoint)

## tl_publisher

### subscriber 
- None

### publisher
- [`/vehicle/traffic_lights[styx_msgs/TrafficLightArray]`](./msg.md#traffic_lights)

# Planning
## Waypoint Loader[waypoint_loader]

### subscriber 
- None

### publisher
- [`/base_waypoints[styx_msgs/Lane]`](./msg.md#base_waypoints)

## Waypoint Updater Node [waypoint_updater]

### subscriber 
- [`/current_pose[geometry_msgs/PoseStamped]`](./msg.md#current_pose)
- [`/base_waypoints[styx_msgs/Lane]`](./msg.md#base_waypoints)
- **ToDo** [`/vehicle/obstacle[geometry_msgs/PoseStamped]`](./msg.md#obstacle)
- **ToDo** [`/traffic_waypoint[std_msgs/Int32]`](./msg.md#traffic_waypoint)

### publisher
- [`/final_waypoints[styx_msgs/Lane]`](./msg.md#final_waypoints)

# Control

## DBW Node[dbw_node]
### subscriber 
- [`/vehicle/dbw_enabled[std_msgs/Bool]`](./msg.md#dbw_enabled)
- **ToDo**[`/twist_cmd[geometry_msgs/TwistStamped]`](./msg.md#twist_cmd)
- **ToDo**[`/current_velocity[geometry_msgs/TwistStamped]`](./msg.md#current_velocity)

### publisher
- [`/vehicle/steering_cmd[dbw_mkz_msgs/ThrottleCmd]`](./msg.md#steering_cmd)
- [`/vehicle/throttle_cmd[dbw_mkz_msgs/SteeringReport]`](./msg.md#throttle_cmd)
- [`/vehicle/brake_cmd[dbw_mkz_msgs/BrakeCmd]`](./msg.md#brake_cmd)

## Waypoint Follower[pure_pursuit]
## subscriber 
- [`/final_waypoints[styx_msgs/Lane]`](./msg.md#final_waypoints)
- [`/current_pose[geometry_msgs/PoseStamped]`](./msg.md#current_pose)
- [`/current_velocity[geometry_msgs/TwistStamped]`](./msg.md#current_velocity)

## publisher
- [`/twist_cmd[geometry_msgs/TwistStamped]`](./msg.md#twist_cmd)
