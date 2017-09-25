# Car/Simulator

## styx_server

### subscriber
- [/vehicle/steering_cmd](./msg.md#steering_cmd)
- [/vehicle/throttle_cmd](./msg.md#throttle_cmd)
- [/vehicle/brake_cmd](./msg.md#brake_cmd)

### publisher
- [/current_pose](./msg.md#current_pose)
- [/current_velocity](./msg.md#current_velocity)
- [/vehicle/steering_report](./msg.md#steering_report)
- [/vehicle/throttle_report](./msg.md#throttle_report)
- [/vehicle/brake_report](./msg.md#brake_report)
- [/vehicle/obstacle](./msg.md#obstacle)
- [/vehicle/obstacle_points](./msg.md#obstacle_points)
- [/vehicle/lidar](./msg.md#lidar)./msg.md#cle/traffic_lights](./msg.md#traffic_lights)
- [/vehicle/dbw_enabled](./msg.md#dbw_enabled)
- [/image_color](./msg.md#image_color)

## camera_info_publisher

### subscriber 
- None

### publisher
- [camera_info])

# Perception

## Traffic Light Detection Node[tl_detctor]

### subscriber 
- [/current_pose](./msg.md#current_pose)
- [/base_waypoints](./msg.md#base_waypoints)
- [/image_color](./msg.md#image_color)

### publisher

- [/vehicle/traffic_lights](./msg.md#traffic_lights)
- [/traffic_waypoint](./msg.md#traffic_waypoint)

## tl_publisher

### subscriber
- None

### publisher
- [/vehicle/traffic_lights](./msg.md#traffic_lights)

# Planning
## Waypoint Loader[waypoint_loader]

### subscriber 
- None

### publisher
- [/base_waypoints](./msg.md#base_waypoints)

## Waypoint Updater Node [waypoint_updater]

### subscriber 
- [/current_pose](./msg.md#current_pose)
- [/base_waypoints](./msg.md#base_waypoints)
- **ToDo**[/vehicle/obtacle]
- **ToDo**[/traffic_waypoint](./msg.md#traffic_waypoint)

### publisher
- [/final_waypoints](./msg.md#final_waypoints)

# Control

## DBW Node[dbw_node]
### subscriber 
- [/vehicle/dbw_enabled](./msg.md#dbw_enabled)
- **ToDo**[/twist_cmd](./msg.md#twist_cmd)
- **ToDo**[/current_velocity](./msg.md#current_velocity)

### publisher
- [/vehicle/steering_cmd](./msg.md#steering_cmd)
- [/vehicle/throttle_cmd](./msg.md#throttle_cmd)
- [/vehicle/brake_cmd](./msg.md#brake_cmd)

## Waypoint Follower[pure_pursuit]
## subscriber 
- [/final_waypoints](./msg.md#final_waypoints)
- [/current_pose](./msg.md#current_pose)
- [/current_velocity](./msg.md#current_velocity)

## publisher
- [/twist_cmd](./msg.md#twist_cmd)
