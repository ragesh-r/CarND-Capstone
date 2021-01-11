import rospy
from yaw_controller import YawController 
from lowpass import LowPassFilter
from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp = 0.3
        ki = 0.1
        kd = 0.001
        mn = 0
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        taus = 0.5
        tss = 0.02
        self.steering_lpf = LowPassFilter(taus, tss)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        
        self.last_time = rospy.get_time()
        
        
    

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        current_vel = self.vel_lpf.filt(current_vel)
        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target vel: {0}".format(linear_vel))
        
        
        
        # steering control
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        #steering = self.steering_lpf.filt(steering)
        #steering = 2
        
        delta_vel = linear_vel - current_vel
        delta_vel = max(self.decel_limit, delta_vel)
        delta_vel = min(delta_vel, self.accel_limit)
        
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(delta_vel, sample_time)
        brake = 0.0
        
        # stop condition
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 700  
            
        elif throttle < 0.1 and delta_vel < 0:
            throttle = 0
            decel = max(delta_vel, self.decel_limit)
            brake = abs(decel)*self.total_mass*self.wheel_radius
            
            
        return throttle, brake, steering
            
            
        
        
