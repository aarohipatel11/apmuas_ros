from apmuas_ros.drone_math import DroneMath


def main() -> None: 
    origin_lat:float = 38.6962509
    origin_lon:float = -94.258194

    waypoint1_lat:float = 38.6962685
    waypoint1_lon:float = -94.2598629

    waypoint2_lat:float = 38.6942107
    waypoint2_lon:float = -94.2599273

    rel_waypoint1_x: float = 0.0
    rel_waypoint1_y:float = 0.0

    rel_waypoint2_x: float = 0.0
    rel_waypoint2_y:float = 0.0

    rel_waypoint1_x, rel_waypoint1_y= DroneMath.geodetic_to_local(origin_lat=origin_lat,
                                                              origin_lon=origin_lon,
                                                              target_lat=waypoint1_lat,
                                                              target_lon=waypoint1_lon)
    
    rel_waypoint2_x, rel_waypoint2_y= DroneMath.geodetic_to_local(origin_lat=origin_lat,
                                                              origin_lon=origin_lon,
                                                              target_lat=waypoint2_lat,
                                                              target_lon=waypoint2_lon)
    
    print("Relative Values: ")
    print("Waypoint 1 Relative Values: ", rel_waypoint1_x, rel_waypoint1_y)
    print("Waypoint 2 Relative Values: ", rel_waypoint2_x, rel_waypoint2_y)

if __name__ == '__main__':
    main()