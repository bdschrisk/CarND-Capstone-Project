from yaw_controller import YawController
from geometry_msgs.msg import PoseStamped, TwistStamped


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        wheel_base = args[0]
        steer_ratio = args[1]
        max_lat_accel = args[2]
        max_steer_angle = args[3]
        min_speed = 20.

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, 
        									max_lat_accel, max_steer_angle)
        
        self.twist_cmd_reveived = TwistStamped()
        self.current_velocity_received = TwistStamped()
        self.throttle_cmd = None
        self.brake_cmd = None
        self.steer_cmd = None


    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        self.twist_cmd_received = args[0]

        self.throttle_cmd = 0.75
        self.brake_cmd = 0.

        self.angle = self.yaw_controller.get_angle(4.)

        linear_velocity = self.twist_cmd_received.twist.linear.x
        angular_velocity = self.twist_cmd_received.twist.angular.x
        self.current_velocity_received = args[1]
        current_velocity = self.current_velocity_received.twist.linear.x
        self.steer_cmd = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        return self.throttle_cmd, self.brake_cmd, self.steer_cmd
