from pymavlink import mavutil
import time
import math
import string
from typing import List, Dict, Any
#TODO: Need to reclass 
#from drone_math import DroneMath
import copy


class DroneCommander():
    #Interface with pymavutil 
    """
    Args:
        master_link (str): The connection string for the drone.
        use_serial (bool): Whether to use a serial connection.
        baud_rate (int): The baud rate for the serial connection.
    """
    def __init__(self,
                 master_link:str='udpin:127.0.0.1:14551',
                 use_serial:bool=False,
                 baud_rate:int=115200)->None:
        self.master_link:str = master_link
        self.use_serial:bool = use_serial
        self.baud_rate:int = baud_rate
        if self.use_serial:
            self.master = mavutil.mavlink_connection(self.master_link, baud=self.baud_rate)
        else:
            # should be UDP connection
            self.master = mavutil.mavlink_connection(self.master_link)
        self.master:mavutil = mavutil.mavlink_connection(self.master_link)
        self.master.wait_heartbeat()
        print('Connected')
    

    def send_waypoint(self,
                    lat_dg:float,
                    long_dg:float,
                    altitude_m:float,
                    hold_time_param:int=0,
                    seq:int=0,
                    radius_converg_param_m:int=5,
                    pass_through_radius_m:int=0,
                    yaw_angle:int=0
                    ) -> bool:
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
        """
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        current = 0  # Not the current mission item (set to 1 if you want immediate execution)
        autocontinue = 1
        self.master.mav.mission_item_send(
            self.master.target_system, self.master.target_component,
            seq, frame, command,
            current, autocontinue,
            hold_time_param, radius_converg_param_m, pass_through_radius_m, yaw_angle,
            lat_dg, long_dg, altitude_m
        )

    def send_loiter_command(self, item: Dict[str, Any],
                       index: int) -> None:
        """
        Args:
            master: The mavutil connection instance.
            item: A dictionary representing a loiter command.

        This function sends a loiter command to the vehicle.

        """
        # So don't freak about this block of code, it's just sending the mission item
        # We're wrapping this inside the mission_item_send api
        # https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-loiter-turns
        self.master.mav.mission_item_send(
            self.master.target_system,
            self.master.target_component,
            int(index),
            item.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
            item['command'],                  # e.g., MAV_CMD_NAV_LOITER_TURNS
            # current (set to 1 for the first item if needed)
            item.get('current', 0),
            item.get('autocontinue', 1),  # autocontinue
            item.get('param1', 0),  # Number of turns
            item.get('param2', 0),  # Nothing
            item.get('param3', 0),  # radius meters CCW is negative
            item.get('param4', 0),  # Nothing
            item['x'],  # latitude (in decimal degrees)
            item['y'],  # longitude (in decimal degrees)
            item['z'],   # altitude (in meters),
        )

    def build_loiter_mission_item(self, 
                                  lat:float, 
                                  lon:float, 
                                  alt:float,
                                  number_of_turns:int,
                                  radius_m:float,
                                  current:int=1
                                  )->Dict[str,Any]:
        mission_item:Dict[str, Any] = {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        'command': mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
        'current': current,         # Mark this as the current mission item
        'autocontinue': 1,
        'param1': number_of_turns,          # Number of loiter turns
        'param2': 0,
        'param3': radius_m,  # Radius in meters, negative is CCW
        'param4': 0,          # Unused (yaw, etc.)
        'lat': lat,     # Latitude in decimal degrees
        'lon': lon,      # Longitude in decimal degrees
        'alt': alt            # Altitude in meters
        }
        return mission_item
    
    def change_airspeed(self,
                        new_airspeed:float,
                        speed_type:int = 0,
                        throttle:int = -1,
                        change_mode:int = 0):
        # speed_type = 0  # 0 for airspeed, 1 for ground speed
        # new_airspeed = 10  # Desired speed in m/s
        # throttle = -1  # Use -1 to indicate no change to throttle
        # change_mode = 0  # 0 = absolute, 1 = relative

        # Send command to change speed
        self.master.mav.command_long_send(
        self.master.target_system, 
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, #confirmation
        speed_type, #Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        new_airspeed, #Speed #m/s
        throttle, #Throttle (-1 indicates no change) % 
        0, 0, 0, 0 #ignore other parameters
        )
        print("changed airspeed")

    def change_roll_contraints(self, 
                                 max_roll_deg:float):
        '''
        This function constrains the roll of the aircraft, overriding whatever the flight controller does. 
        To check if this function worked, run the script with this function, and then in Mission Planner
        click on config at the top, then full parameter list, and search for whatever parameter you are constraining. 
        In this case, it is the 'ROLL_LIMIT_DEG' parameter, and see if the value changed to whatever you wanted through the script.
        https://mavlink.io/en/services/parameter.html 
        '''

        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'ROLL_LIMIT_DEG',
            max_roll_deg,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def upload_mission(self,
                       mission_items:List[Dict[str, Any]])->bool:
        '''
        basic idea is to clear the mission cache,
        set the length of the mission list, and then send each mission item
        '''
        #clears the mission cache
        print("Clearing mission...")
        self.master.mav.mission_clear_all_send(
            self.master.target_system, self.master.target_component)
        
        #set the length of the mission list
        mission_count:int = len(mission_items)
        print(f"Sending mission count: {mission_count}")
        self.master.mav.mission_count_send(self.master.target_system,
                                  self.master.target_component,
                                  mission_count,
                                  mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        
        for i in range(mission_count):
            print(f"Waiting for mission request for item {i}...")
            req = self.master.recv_match(
                type=['MISSION_REQUEST'], blocking=True, timeout=True)
            if req is None:
                print(f"Timeout waiting for mission request for item {i}")
                return False
            
            item = mission_items[i]
            print(f"Sending mission item {i}...")
            time.sleep(1)
            self.send_loiter_command(item, req.seq)
        
        #wait for mission acknowledgement
        print("Waiting for mission acknowledgment...")
        ack = self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=10)
        return True
    
    def read_mission_items(self) -> List[Dict[str, Any]]:
        """
        We are basically doing the inverse of the upload_mission function
        - Request the mission 
        - Get the mission count
        - Request each mission item
        - Return the mission items
        """
        self.master.mav.mission_request_list_send(
            self.master.target_system, self.master.target_component
        )
    
        mission_count_msg = self.master.recv_match(
            type='MISSION_COUNT', blocking=True, timeout=5
        )
                
        if mission_count_msg is None:
            print("Timeout: No mission count received")
            return []
        
        mission_count = mission_count_msg.count
        print("Mission count:", mission_count)
        
        mission_items = []  
        
        for i in range(mission_count):
            # Request the mission item with index i.
            self.master.mav.mission_request_send(
                self.master.target_system, self.master.target_component, i
            )
            
            # Wait for the mission item message.
            mission_item_msg = self.master.recv_match(
                type='MISSION_ITEM', blocking=True, timeout=5
            )
            if mission_item_msg is None:
                print(f"Timeout: No mission item received for index {i}")
                return mission_items
            
            mission_item_dict = mission_item_msg.to_dict()
            mission_items.append(mission_item_dict)
        
        return mission_items
    
    def upload_mission2(self, mission_items: List[Dict[str, Any]]) -> bool:
        """
        Uploads a mission to the drone, ensuring each mission item is sent correctly.
        """
        # Clear previous mission
        print("Clearing mission...")
        self.master.mav.mission_clear_all_send(
            self.master.target_system, self.master.target_component
        )

        # Set mission count
        mission_count:int = len(mission_items)
        print(f"Sending mission count: {mission_count}")
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            mission_count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        for i, item in enumerate(mission_items):
            print(f"Waiting for mission request for item {i}...")

            req = self.master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=5)
            if req is None:
                print(f"Timeout waiting for mission request for item {i}")
                return False
            # if req.seq != i:
            #     print(f"Unexpected mission request sequence: expected {i}, got {req.seq}")
            #     return False

            # Send the mission item exactly as specified
            print(f"Sending mission item {i}...")
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                int(i),  # Sequence number
                item.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                item.get("command", mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                item.get("current", 0),
                item.get("autocontinue", 1),
                item.get("param1", 0),
                item.get("param2", 0),
                item.get("param3", 0),
                item.get("param4", 0),
                item.get("x", 0),  # Latitude
                item.get("y", 0),  # Longitude
                item.get("z", 0)   # Altitude
            )
            time.sleep(1)  # Small delay to ensure processing

        # Wait for mission acknowledgment
        print("Waiting for mission acknowledgment...")
        ack = self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=10)
        if ack is None:
            print("No mission acknowledgment received!")
            return False
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"Mission not accepted, received ack type: {ack.type}")
            return False

        #print("Mission uploaded successfully!")
        return True
    
    def check_rc_channel(self, rc_channel=7) -> int:
        #Get the number value of specified channel --> useful when testing offboard controls
        message = self.master.recv_match(type='RC_CHANNELS', blocking=True)
        if message:
            channel_value = getattr(message, f'chan{rc_channel}_raw', None)
            if channel_value is not None:
                return channel_value
        return None
    
    def is_trigger_on(self, rc_channel=7, channel_threshold=1550) -> bool:
        #return the state of the switch depending on the threshold specified
        value = self.check_rc_channel(rc_channel=rc_channel)
        if value is None:
            return False
        elif value >= channel_threshold:
            return True
        else:
            return False
        
    def change_mode(self, mode:string) -> None:
        '''
        This is usually done through the ground control station, but it is here as an option to change the mode to whatever the specified mode is. 
        '''
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    
    def arm_vehicle(self) -> None:
        '''
        This is usually done through the ground control station, but it is here as an option to arm the vehicle. 
        Changing param 1 to be 0, disarms the vehicle.
        '''
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 
            1, #param 1 --> 0: disarm, 1: arm
            0, 0, 0, 0, 0, 0
        )
        time.sleep(2)



    def create_updated_waypoint_list(self, 
                                     original_waypoints:List[Dict[str, Any]],
                                     max_roll:float, 
                                     waypoint_command:int, 
                                     loiter_command:int,
                                     camera_range_m:float,
                                     alt_max_lim:float,
                                     alt_min_lim:float,
                                     num_loiter:int) -> List[Dict[str, Any]]:
        '''
        This fuction creates an updated waypoint list from different specifications from the user. (Passed through as parameters to this function)
        Goes through the mission items in the list and if it is either a waypoint command or loiter command, it makes a copy of the item, makes some alterations and then 
        appends it to the updated waypoint/new list, otherwise, it just keeps it as is, and adds its copy to the updated waypoint/new list. 
        Alterations to parameters are mostly to loiter radius, altitude, and other math which is defined in the DroneMath class in drone_math.py. 
        '''
        updated_waypoints: List[Dict[str, Any]] = []
        for waypoint in original_waypoints:
            #print(waypoint)
            current_waypoint = copy.deepcopy(waypoint)
            if current_waypoint['command'] == waypoint_command:
                current_waypoint['command'] = loiter_command
                current_waypoint['param1'] = num_loiter
                current_waypoint['param3'] = DroneMath.compute_loiter_radius_m(mount_angle_phi_deg=max_roll,
                                                                        cam_range_m=camera_range_m,
                                                                        ccw_loiter=1,
                                                                        roll_limit_deg=max_roll)
                current_waypoint['z'] = DroneMath.compute_altitude_m(cam_range_m_alt=camera_range_m,
                                                                     max_roll_tan=max_roll,
                                                                     max_alt_m=alt_max_lim,
                                                                     min_alt_m=alt_min_lim)
                updated_waypoints.append(current_waypoint)

            elif current_waypoint['command'] == loiter_command:
                current_waypoint['param3'] = DroneMath.compute_loiter_radius_m(mount_angle_phi_deg=max_roll,
                                                                        cam_range_m=camera_range_m,
                                                                        ccw_loiter=1,
                                                                        roll_limit_deg=max_roll)
                current_waypoint['z'] = DroneMath.compute_altitude_m(cam_range_m_alt=camera_range_m,
                                                                     max_roll_tan=max_roll,
                                                                     max_alt_m=alt_max_lim,
                                                                     min_alt_m=alt_min_lim)
                current_waypoint['param1'] = num_loiter
                updated_waypoints.append(current_waypoint)

            else: 
                updated_waypoints.append(current_waypoint)
        assert len(original_waypoints) == len(updated_waypoints)
        return updated_waypoints
        #print("length of original waypoints and new ", len(orig_waypoints), len(updated_waypoints))


    def read_switch(self,
                    switch_threshold:float,
                    previous_channel_value,
                    original_waypoints:List[Dict[str, Any]],
                    updated_waypoints: List[Dict[str, Any]],
                    channel_number:int) -> None:
        '''
        Note: upload_mission and upload_mission2 do not have huge differences, and can be used interchangably if you want. 
        Used when testing with hardware in the loop or during the actual test.
        This function checks the state of the controller switch and compares it to the defined threshold. 
        If it is the same as the previous, it keeps checking (while loop) until it notices a difference. 
        A large value, above the threshold, indicates that the switch is on, and sends the original waypoint list just how the user put the values in. 
        A small value, below the threshold, meaning the switch is off, will send the changed waypoints and you can hit "read" on Mission Planner, and the changed values will appear. 
        '''
        
        while True:
            try:
                current_channel_value = self.check_rc_channel(rc_channel=channel_number)
                print("current channel value", current_channel_value)
                if current_channel_value == previous_channel_value:
                    continue
                else:
                    previous_channel_value = current_channel_value

                if current_channel_value >= switch_threshold:
                    print("Sending the original waypoints")
                    self.upload_mission(mission_items=original_waypoints)
                elif current_channel_value < switch_threshold:
                    print("Sending the updated waypoints")
                    self.upload_mission2(mission_items=updated_waypoints)
                time.sleep(0.1)

            except KeyboardInterrupt:
                print("Exiting...")
                break       

    def software_switch_test(self,
                    switch_threshold:float,
                    previous_channel_value,
                    original_waypoints:List[Dict[str, Any]],
                    updated_waypoints: List[Dict[str, Any]]) -> None:
        '''
        Note: upload_mission and upload_mission2 do not have huge differences, and can be used interchangably if you want. 
        Used when testing with software in the loop or with the simulator.
        This function checks the state of the controller switch and compares it to the defined threshold. 
        If it is the same as the previous, it keeps checking (while loop) until it notices a difference. 
        A small value, below the threshold, indicates that the switch is off, and sends the original waypoint list just how the user put the values in. 
        A large value, above the threshold, meaning the switch is on, will send the changed waypoints and you can hit "read" on Mission Planner, and the changed values will appear. 
        '''
        while True: 
            try:
                input_val = int(input("Enter a value (0-2): "))
                if input_val == previous_channel_value:
                    continue
                else:
                    previous_channel_value = input_val

                print("length of original waypoints and new ", len(original_waypoints), len(updated_waypoints))
                if input_val < switch_threshold:
                    print("Sending original waypoints")
                    print(updated_waypoints)
                    self.upload_mission(mission_items=original_waypoints)
                elif input_val >= switch_threshold:
                    print("Sending updated waypoints")
                    print(updated_waypoints)
                    self.upload_mission2(mission_items=updated_waypoints)
                else:
                    print("doing nothing")
                time.sleep(0.1)

            except KeyboardInterrupt:
                print("Exiting...")
                break