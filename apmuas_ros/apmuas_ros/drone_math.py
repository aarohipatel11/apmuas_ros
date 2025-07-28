import math
import numpy as np
from scipy.constants import g
from typing import List, Tuple


# Earth radius in meters
EARTH_RADIUS = 6371000  


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

def deg2rad(deg: float) -> float:
    return deg * math.pi / 180 


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float: 
    """
    Calculates the Haversine distance between two points, longitude and latitude (in decimal degrees).
    
    Args: lat1 (float): Latitude of the first point.
          lon1 (float): Longitude of the first point.
          lat2 (float): Latitude of the second point.
          lon2 (float): Longitude of the second point.

    Returns:
        float: The Haversine distance in meters between the two points.
    
    """
    dlat: float = deg2rad(lat2 - lat1)
    dlon: float = deg2rad(lon2 - lon1)
    lat1 = deg2rad(lat1)
    lat2 = deg2rad(lat2)

    a: float = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c: float = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c


def initial_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculates the initial bearing (direction) from one point to another.
    Longitude and latitude are in decimal degrees.

    Args:
        lat1 (float): Latitude of the first point.
        lon1 (float): Longitude of the first point.
        lat2 (float): Latitude of the second point.
        lon2 (float): Longitude of the second point.

    Returns:
        float: The initial bearing in radians from the first point to the second point.
    
    """
    lat1 = deg2rad(lat1)
    lat2 = deg2rad(lat2)
    dlon = deg2rad(lon2 - lon1)

    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    return math.atan2(x, y)


def geodetic_to_cartesian(origin_lat: float, origin_lon: float, target_lat: float, target_lon: float) -> Tuple[float, float]:
    """
    Converts geodetic coordinates (latitude, longitude) to Cartesian coordinates (x, y) relative to an origin point.
    Longitude and latitude are in decimal degrees.

    Args:
        origin_lat (float): Latitude of the origin point.
        origin_lon (float): Longitude of the origin point.
        target_lat (float): Latitude of the target point.
        target_lon (float): Longitude of the target point.

    Returns:
        tuple: A coordinate pair (x, y) representing the Cartesian coordinates relative to the origin point.
    
    """
    distance: float = haversine_distance(origin_lat, origin_lon, target_lat, target_lon)
    bearing: float = initial_bearing(origin_lat, origin_lon, target_lat, target_lon)
    x: float = distance * math.sin(bearing)
    y: float = distance * math.cos(bearing)
    return x, y


def convert_all_to_cartesian(coords: List[List[float]]) -> List[List[float]]: 
    """
    Converts a list of longitude/latitude coordinates to local Cartesian coordinates, using the first coordinate as the origin.
    Longitude and latitude are in decimal degrees.
    Args:
        coords (List[List[float]]): A list of lists, where each inner list contains a latitude and longitude pair.

    Returns:
        List[List[float]]: A list of Cartesian coordinates (x, y) relative to the first coordinate in the list (the origin).
    
    """
    if not coords:
        return []

    origin_lat: float = coords[0][0] 
    origin_lon: float = coords[0][1]

    cartesian_coords: List[List[float]] = [] 

    for lat, lon, altitude in coords: 
        x, y = geodetic_to_cartesian(origin_lat, origin_lon, lat, lon)
        cartesian_coords.append([x, y, altitude]) 

    return cartesian_coords
