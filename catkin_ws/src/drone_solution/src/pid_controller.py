from __future__ import print_function


def clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


class PID(object):
    def __init__(self, Kp, Ki, Kd, output_limits):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.last_error = 0
        self.output_limits = output_limits

    def __call__(self, error, dt):
        self.integral = clamp(self.Ki * error * dt, self.output_limits)

        # Compute final output
        output = clamp(self.Kp * error + self.integral + -self.Kd * (error - self.last_error) / dt, self.output_limits)

        # Keep track of state
        self.last_error = error

        return output
