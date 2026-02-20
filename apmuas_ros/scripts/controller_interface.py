# This is the Client Interface for the Adapter Design
# It will define what the Client (GuidanceAlgorithmService) expects

from abc import ABC, abstractmethod
from typing import Any, List

class ControllerInterface(ABC):
    @abstractmethod
    def get_commands(self, current_state: List[float], target_state: List[float]) -> Any:
        """
        Calculate control commands to navigate from current state to target state.
        
        Args:
            current_state: [x, y, z, roll, pitch, yaw, airspeed] - where drone currently IS
            target_state: [x, y, z] - desired waypoint the drone WANTS TO REACH
            
        Returns:
            Control commands (CtlTraj)
        """
        pass

    