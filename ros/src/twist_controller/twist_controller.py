from pid import PID as PID
from yaw_controller import YawController as YAWController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband, decel_limit,accel_limit,
                wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YAWController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)
        
        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0
        mx = 0.5
        self.throttle_controller = PID(kp,ki,kd,mn,mx)
        
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau,ts)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.break_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()
        

    def control(self, dbw_enabled,current_vel,linear_vel,angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        brake = 0.
        throttle = 0.
        steering = 0.
        
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.,0.,0.
        
        filtered_current_vel = self.vel_lpf.filt(current_vel)
        #filtered_current_vel = current_vel
        steering = self.yaw_controller.get_steering(linear_vel,angular_vel,filtered_current_vel)
        vel_error = linear_vel - filtered_current_vel
        self.last_vel = filtered_current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error,sample_time)    
        
        if linear_vel == 0. and current_vel <0:
            throttle = 0
            brake = 400
        elif throttle <.1 and vel_error<0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius        
        rospy.loginfo("Steerign info in twist controller {}".format(steering))
        return throttle,brake,steering
