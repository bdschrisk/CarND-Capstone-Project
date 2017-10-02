
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.twist_cmd_reveived = None
        self.throttle_cmd = None
        self.brake_cmd = None
        self.steer_cmd = None


    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        self.twist_cmd_received = args[0]

        self.throttle_cmd = 0.75
        self.brake_cmd = 0.
        self.steer_cmd = self.twist_cmd_received.twist.linear.x/20.

        return self.throttle_cmd, self.brake_cmd, self.steer_cmd
