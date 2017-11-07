
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from pid import PID
from yaw_controller import YawController


class Controller(object):
    # def __init__(self, *args, **kwargs):
    #     # TODO: Implement
    #     pass

    def __init__(self, brake_deadband, decel_limit, accel_limit, wheel_base, steer_ratio,
            max_lat_accel, max_steer_angle):


        # vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        # fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        # wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.min_speed = 1
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

        self.pid_throttle = PID(0.5, 0.1, 0.001, -1, 1)
        self.pid_brake = PID(0.5, 0.1, 0.001, 0, 1)

        # self.brake_torque = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * deceleration * wheel_radius
    # TODO: Implement
    pass	

    def control(self, desired_twist, current_twist, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        desired_vel_linear = desired_twist.linear.x
        current_vel_linear = current_twist.linear.x
        desired_vel_angular = desired_twist.angular.z


        throttle = self.pid_throttle.step((desired_vel_linear - current_vel_linear), dt)
        steer = self.yaw_controller.get_steering(desired_vel_linear, desired_vel_angular, current_vel_linear)

        if throttle < 0:
            throttle = 0
            brake = -throttle
        else:
            brake = 0

        # brake = self.pid_brake.step((-desired_vel + current_vel), dt)



        return throttle, brake, steer
        # return 1., 0., 0.
