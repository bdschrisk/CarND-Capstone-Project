
# topic:/vehicle/dbw_enabled
## std_msgs/Bool

# topic:/traffic_waypoint 
## std_msgs/Int32

# topic:/vehicle/dbw_enabled
## std_msgs/Bool

# topic:/vehicle/brake_report
## std_msgs/Float32

# topic:/vehicle/throttle_report
## std_msgs/Float32 

# topic:/base_waypoints
# topic:/final_waypoints 
## styx_msgs/Lane
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
~~~

# topic:/vehicle/traffic_lights
## styx_msgs/TrafficLightArray
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/TrafficLight[] lights
  uint8 UNKNOWN=4
  uint8 GREEN=2
  uint8 YELLOW=1
  uint8 RED=0
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  uint8 state
~~~

# topic:/current_velocity
# topic:/twist_cmd

## geometry_msgs/TwistStamped
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
~~~

# topic:/current_pose
# topic:/vehicle/obstacle

## geometry_msgs/PoseStamped 
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
~~~

# topic:/vehicle/obstacle_points
# topic:/vehicle/lidar

## sensor_msgs/PointCloud2
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
sensor_msgs/PointField[] fields
  uint8 INT8=1
  uint8 UINT8=2
  uint8 INT16=3
  uint8 UINT16=4
  uint8 INT32=5
  uint8 UINT32=6
  uint8 FLOAT32=7
  uint8 FLOAT64=8
  string name
  uint32 offset
  uint8 datatype
  uint32 count
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
~~~

# topic:/image_color
## sensor_msgs/Image
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
~~~

# topic:/tf
## tf2_msgs/TFMessage
~~~
geometry_msgs/TransformStamped[] transforms
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
~~~

<a name="test"></a>
# topic:/vehicle/steering_cmd
## dbw_mkz_msgs/SteeringCmd

~~~
float32 steering_wheel_angle_cmd
float32 steering_wheel_angle_velocity
bool enable
bool clear
bool ignore
bool quiet
uint8 count
~~~

# topic:/vehicle/brake_cmd
## dbw_mkz_msgsBrakeCmd
~~~
uint8 CMD_NONE=0
uint8 CMD_PEDAL=1
uint8 CMD_PERCENT=2
uint8 CMD_TORQUE=3
float32 TORQUE_BOO=520
float32 TORQUE_MAX=3412
float32 pedal_cmd
uint8 pedal_cmd_type
bool boo_cmd
bool enable
bool clear
bool ignore
uint8 count
~~~

# topic:/vehicle/throttle_cmd
## dbw_mkz_msgs/ThrottleCmd
~~~
uint8 CMD_NONE=0
uint8 CMD_PEDAL=1
uint8 CMD_PERCENT=2
float32 pedal_cmd
uint8 pedal_cmd_type
bool enable
bool clear
bool ignore
uint8 count
~~~

# /vehicle/steering_report
## dbw_mkz_msgs/SteeringReport
~~~
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 steering_wheel_angle
float32 steering_wheel_angle_cmd
float32 steering_wheel_torque
float32 speed
bool enabled
bool override
bool timeout
bool fault_wdc
bool fault_bus1
bool fault_bus2
bool fault_calibration
~~~
