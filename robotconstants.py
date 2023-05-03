import math

class RobotConstants:
    dir_pid_prop_constant = 20
    dir_pid_integ_constant = 0
    dir_pid_deriv_constant = 0

    # Use if we want to implement speed PID
    spd_pid_prop_constant = 0.5
    spd_pid_integ_constant = 0
    spd_pid_deriv_constant = 0

    distancePerTick = (math.pi * 0.07) / (12 * 120)
    intendedDistance = 2.0
