import math
import numpy as np
from scipy.constants import g

class DroneMath():
    def __init__(self):
        pass       
    '''
    Math class that computes different values like loiter radius, velocity, and altitude for the drone. 
    ''' 
    
    def compute_loiter_radius_m(mount_angle_phi_deg:float, 
                                cam_range_m:float, 
                                ccw_loiter:bool,
                                roll_limit_deg:float) -> float:
        neg_roll_deg = ((-1) * roll_limit_deg)
        if mount_angle_phi_deg < neg_roll_deg:
            mount_angle_phi_deg = neg_roll_deg
        elif mount_angle_phi_deg > roll_limit_deg:
            mount_angle_phi_deg = roll_limit_deg
        
        if ccw_loiter:
            return (-1)*(math.sin(math.radians(mount_angle_phi_deg)) * cam_range_m)
        return (math.sin(math.radians(mount_angle_phi_deg)) * cam_range_m)
    
    
    def calc_velocity(mount_angle_phi_deg:float, cam_range_m:float, ccw_loiter:bool) -> float:
        loiter_radius:float = DroneMath.compute_loiter_radius_m(mount_angle_phi_deg=mount_angle_phi_deg,
                                                                cam_range_m=cam_range_m,
                                                                ccw_loiter=ccw_loiter)
        mount_angle_rad = math.radians(mount_angle_phi_deg)
        if ccw_loiter:
            return math.sqrt((-1)*(loiter_radius) * g * math.tan(mount_angle_rad))
        return math.sqrt(loiter_radius * g * math.tan(mount_angle_rad))
    
    
    def compute_altitude_m(cam_range_m_alt:float,
                           max_roll_tan:float,
                           max_alt_m:float,
                           min_alt_m:float) -> float:
        max_roll_rad = np.deg2rad(max_roll_tan)
        calc_alt = (cam_range_m_alt)*(math.cos(max_roll_rad))
        if calc_alt > max_alt_m:
            return max_alt_m
        elif calc_alt < min_alt_m:
            return min_alt_m
        else:
            return calc_alt


    def realtime_loiter_radius(mount_angle_phi_deg:float, 
                                cam_range_m:float,
                                roll_limit_deg:float) -> float:
        neg_roll_deg = ((-1) * roll_limit_deg)
        if mount_angle_phi_deg < neg_roll_deg:
            mount_angle_phi_deg = neg_roll_deg
        elif mount_angle_phi_deg > roll_limit_deg:
            mount_angle_phi_deg = roll_limit_deg
        
        return (math.sin(math.radians(mount_angle_phi_deg)) * cam_range_m)
    

    def calculate_loiter_time(num_loiters:int, loiter_radius: float, aircraft_velocity_mps: float) -> float: 
        loiter_circumference: float = (2) * (np.pi) * (loiter_radius)
        total_distance: float = loiter_circumference * num_loiters
        loiter_time = (total_distance) / aircraft_velocity_mps
        return loiter_time