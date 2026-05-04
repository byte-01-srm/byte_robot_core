import math
import time
from config.robot_config import LEG_ORDER, SIT_COORDS

class TrotGaitEngine:
    def __init__(self):
        # Gait Parameters (aligned to your +X=fwd, +Z=up system)
        self.STRIDE_X = 8.0      # Forward travel (cm)
        self.LIFT_Z = 5.0        # Step height (cm)
        self.GROUND_PUSH = 0.2   # Downward stance push (cm)
        self.FREQ = 2.0          # Hz
        self.UPDATE_HZ = 100.0
        
        self.PHASE_OFFSETS = {
            "fl": 0.0, "br": 0.0,  # Pair A
            "fr": 0.5, "bl": 0.5   # Pair B
        }

    def calculate_step_deltas(self, phase, current_positions):
        """
        Calculates dx, dy, dz relative to the CURRENT position of each leg.
        """
        payload = {}
        for leg in LEG_ORDER:
            # phi is the local phase for this leg
            phi = (phase + self.PHASE_OFFSETS[leg]) % 1.0
            
            # Base height (Z) from your sitting/standing baseline
            # Since walking happens from "Stand", we use Stand's Z (-14)
            z0 = -14.0 
            x0 = 0.0
            y0 = 0.0

            if phi < 0.5: # Swing Phase
                s = phi / 0.5
                # Cycloidal trajectory for Forward (X)
                dx_abs = -self.STRIDE_X/2 + self.STRIDE_X * (s - math.sin(2*math.pi*s)/(2*math.pi))
                # Lift (Z)
                dz_abs = z0 + self.LIFT_Z * (1 - math.cos(2*math.pi*s))
            else: # Stance Phase
                s = (phi - 0.5) / 0.5
                # Linear pushback for Forward (X)
                dx_abs = self.STRIDE_X/2 - self.STRIDE_X * s
                # Small ground push (Z)
                dz_abs = z0 - self.GROUND_PUSH * math.sin(math.pi * s)

            # Calculate relative delta: (Target_Absolute - Current_State_Manager_Absolute)
            # This ensures main_with_ik gets the correct relative shift[cite: 4, 8]
            dx = dx_abs - current_positions[leg][0]
            dy = y0 - current_positions[leg][1]
            dz = dz_abs - current_positions[leg][2]

            payload[leg] = [dx, dy, dz]
            
        return payload