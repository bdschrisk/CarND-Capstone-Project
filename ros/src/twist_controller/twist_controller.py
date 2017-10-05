GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, pid, low_pass_filter):

        self.yaw_controller = yaw_controller
        self.pid = pid
        self.low_pass_filter = low_pass_filter

        # TODO: Implement
        self.twist_cmd_reveived = None
        self.throttle_cmd = None
        self.brake_cmd = None
        self.steer_cmd = None

    def control(self, proposed_linear_vel, proposed_ang_vel, current_linear_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        steering = self.yaw_controller.get_steering(proposed_linear_vel, proposed_ang_vel, current_linear_vel)

        #TODO: calculate throttle command and brake using PID maybe?

        self.throttle_cmd = 0.2
        self.brake_cmd = 0.
        self.steer_cmd = steering

        return self.throttle_cmd, self.brake_cmd, self.steer_cmd
