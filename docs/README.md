# System Overview
[ALL Nodelist](./node.md)

![](./final-project-ros-graph-v2.png)

# Ros debug commands

- rostopic pub
~~~~
[throttle]
rostopic pub -r 1 /vehicle/throttle_cmd dbw_mkz_msgs/ThrottleCmd -- "{pedal_cmd: 1.0, pedal_cmd_type: 2, enable: 1}"

[brake]
rostopic pub -r 1 /vehicle/brake_cmd dbw_mkz_msgs/BrakeCmd -- "{pedal_cmd: 1.0, pedal_cmd_type: 3, enable: 1}"
~~~~
