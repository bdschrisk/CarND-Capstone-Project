
# Car/Simulator

## styx_server

### subscriber
- [/vehicle/steering_cmd](./msg.md#topic:/vehicle/steering_cmd])
- /vehicle/throttle_cmd
- /vehicle/brake_cmd

### publisher
- /current_pose
- /current_velocity
- /vehicle/steering_report
- /vehicle/throttle_report
- /vehicle/brake_report
- /vehicle/obstacle'
- /vehicle/obstacle_points
- /vehicle/lidar
- /vehicle/traffic_lights
- /vehicle/dbw_enabled
- /image_color

## camera_info_publisher

### subscriber 
- None

### publisher
- camera_info

# Perception

## Traffic Light Detection Node[tl_detctor]

### subscriber 
- /current_pose
- /base_waypoints
- /image_color

### publisher

- /vehicle/traffic_lights
- /traffic_waypoint

## tl_publisher

### subscriber
- None

### publisher
- /vehicle/traffic_lights

# Planning
## Waypoint Loader[waypoint_loader]

### subscriber 
- None

### publisher
- /base_waypoints

## Waypoint Updater Node [waypoint_updater]

## subscriber 
- /current_pose
- /base_waypoints
- [todo]/obtacle_waypoint
- [todo]/traffic_waypoint

## publisher
- /final_waypoints

# Control

## DBW Node[dbw_node]
### subscriber 
- /vehicle/dbw_enabled
- [todo]/twist_cmd
- [todo]/current_velocity

### publisher
- /vehicle/steering_cmd
- /vehicle/throttle_cmd
- /vehicle/brake_cmd

## Waypoint Follower[pure_pursuit]
## subscriber 
- /final_waypoints
- /current_pose
- /current_velocity

## publisher
- /twist_cmd


