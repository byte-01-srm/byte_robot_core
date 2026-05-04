#!/usr/bin/env python3
import math 
import time
from mpu_leveling import MPULeveler
from config.robot_config import LEG_ORDER

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS  (from trot_gait_node.py — tune these)
# ─────────────────────────────────────────────────────────────────────────────
STRIDE_LENGTH_X  = 10.0     # cm  — S,  total fore-aft foot travel per cycle
STRIDE_LENGTH_Y  = 4.0     # cm  — lateral stride for strafe
GAIT_SPEED_DEG_PER_S = 800.0   # deg/s — used during gait frames
LIFT_HEIGHT = 7.0                  # max lift during swing
PUSH_DEPTH = 1.0                   # max push down during stance

# ─────────────────────────────────────────────────────────────────────────────
# PHASE OFFSETS  (trot diagonal pairs)
# ─────────────────────────────────────────────────────────────────────────────
PHASE_OFFSET = {
    "fl": 0.0,   # ─┐ Pair A start in swing
    "br": 0.0,   # ─┘
    "fr": 0.5,   # ─┐ Pair B start in stance
    "bl": 0.5,   # ─┘
}


# ─────────────────────────────────────────────────────────────────────────────
# RUNTIME STATE
# ─────────────────────────────────────────────────────────────────────────────  
# leveler = MPULeveler()


class TrotGaitController:
    def __init__(self, leg : str, axis: str, direction: int):
        self.leg = leg
        self.axis = axis
        self.direction = direction

        self.x_com = 0.0
        self.z_com = 0.0

        self.phase_off = PHASE_OFFSET[leg]      # phase offset for this leg

        self.STRIDE_LENGTH  = STRIDE_LENGTH_X if self.axis == 'x' else STRIDE_LENGTH_Y
        self.STRIDE_LENGTH *= self.direction

        self.last_pos = [0.0, 0.0, 0.0]         # last foot position for smooth transitions
        self.x_start = 0.0                            # initial x_start based on phase offset

        if self.phase_off == 0.0:
            # starts in Swing
            self.x_start = -self.STRIDE_LENGTH / 2  # start at back of stride for smooth lift
            self.last_pos[0] = self.x_start # start at back of stride for smooth lift

        else:
            # starts in Stance
            self.x_start = self.STRIDE_LENGTH / 2   # start at front of stride for smooth push
            self.last_pos[0] = self.x_start  # start at front of stride for smooth push 



    def calculate_cycloidal_path(self, phi_swing):
        """
        phi_swing: 0.0 to 1.0
        X: -5 -> 5 | Z: 0 -> 4 -> 0
        """
        # Explicit bounds for Swing
        x_start_swing = -(self.STRIDE_LENGTH) / 2
        total_dist = (self.STRIDE_LENGTH)
        
        # X moves forward from -5 to 5
        self.x_com = x_start_swing + total_dist * (phi_swing - (1 / (2 * math.pi)) * math.sin(2 * math.pi * phi_swing))
        
        # Z lifts UP to +LIFT_HEIGHT
        self.z_com = (LIFT_HEIGHT / 2) * (1 - math.cos(2 * math.pi * phi_swing))

    def calculate_sinusoidal_path(self, phi_stance):
        """
        phi_stance: 0.0 to 1.0
        X: 5 -> -5 | Z: 0 -> -5 -> 0
        """
        # Explicit bounds for Stance
        x_start_stance = (self.STRIDE_LENGTH) / 2
        total_dist = -(self.STRIDE_LENGTH) # Negative to move backward
        
        # X moves backward from 5 to -5
        self.x_com = x_start_stance + (total_dist * phi_stance)
        
        # Z pushes DOWN to -PUSH_DEPTH
        # Using sin(pi * phi) because it is 0 at phi=0 and phi=1, and 1 at phi=0.5
        self.z_com = -PUSH_DEPTH * math.sin(math.pi * phi_stance)
        

    # ─────────────────────────────────────────────────────────────────────────────
    # PHASE-BASED FOOT POSITION  (core logic)
    # ─────────────────────────────────────────────────────────────────────────────
    def _foot_pos_phase(self, phase: float) -> list:

        phi = (phase + self.phase_off) % 1.0

        self.x_com = 0.0 
        self.z_com = 0.0

        if phi <= 0.5:
            # ── Swing (0.0 to 0.5) ──────────────────────────────────────────
            phi_swing = phi / 0.5               # Normalize phi to 0.0 - 1.0 for the swing calculation
            self.calculate_cycloidal_path(phi_swing)
            swing_stance = "Swing "
        else:
            # ── Stance (0.5 to 1.0) ─────────────────────────────────────────
            phi_stance = (phi - 0.5) / 0.5      # Normalize phi to 0.0 - 1.0 for the stance calculation
            self.calculate_sinusoidal_path(phi_stance)
            swing_stance = "Stance"

        npx  = self.x_com 
        npz  = self.z_com
        
        if self.axis == 'y': 
            npy = npx
            npx = 0
        else:
            npy = 0

        dx, dy, dz = npx - self.last_pos[0], npy - self.last_pos[1], npz - self.last_pos[2]

        print(f" Leg {self.leg} {swing_stance}  | Phase: {phase:.2f} | x, y, z: {[self.x_com, self.z_com]} | TRANSFORMED x,y,z: {[dx, dy, dz]} ")
        self.last_pos = [npx, npy, npz]  # Update last position for smooth transitions
        return [dx, dy, dz]

# ─────────────────────────────────────────────────────────────────────────────
# CORE GAIT RUNNER
# ─────────────────────────────────────────────────────────────────────────────
def _run_trot_gait_cycle(axis: str, direction: int):
    """ 
    Always completes a full cycle before stopping — all feet land cleanly.
    """
    # UPDATE_HZ      = 100.0   # Hz  — send rate
    # GAIT_FREQUENCY = 2.0     # Hz  — full cycle rate
    # dt          = 1.0 / UPDATE_HZ
    # phase_step  = GAIT_FREQUENCY / UPDATE_HZ
    # phase       = 0.0 

    GAIT_SPEED_DEG_PER_S = 500.0
    time = 0.2
    dt = 0.01
    no_of_updates = int(time / dt)
    phase_step = 1.0 / no_of_updates
    phase = 0.0

    # create a controller instance for each leg and calculate foot positions
    controller = {}  # reset controller dict each loop to ensure clean state
    for leg in LEG_ORDER:
        controller[leg] = TrotGaitController(leg, axis, direction)

    while phase < 1.0:
        payload = {}
        for leg in LEG_ORDER:
            x, y, z = controller[leg]._foot_pos_phase(phase)
            payload[leg] = [x, y, z]
        
        print("\n")
        time.sleep(dt)
        phase += phase_step

    phase = 0.0 # reset phase to ensure clean cycles


def main():
    # Example: Run 3 forward trot cycles
    _run_trot_gait_cycle(axis='x', direction=1)


if __name__ == "__main__":
    main()