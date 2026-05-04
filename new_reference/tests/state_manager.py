import pickle
import socket
import time
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER

class QuadrupedStateManager:
    def __init__(self):
        # Internal tracking of the current position (starts at sit: 0,0,0)
        self.current_positions = {leg: [0.0, 0.0, 0.0] for leg in LEG_ORDER}
        
        # Define Coordinate targets for specific states
        self.states = {
            "sit":   {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "stand": {"dx": 0.0, "dy": 0.0, "dz": -14.0},
        }

    def change_state(self, target_state: str):
        """API-like function to trigger a state transition."""
        if target_state not in self.states:
            print(f"Error: State '{target_state}' not defined.")
            return

        target = self.states[target_state]
        print(f"\nTransitioning to: {target_state.upper()}")

        # Calculate the delta needed to get from CURRENT to TARGET
        # main_with_ik.py adds these deltas to its internal baseline[cite: 8]
        payload_deltas = {}
        
        for leg in LEG_ORDER:
            curr_x, curr_y, curr_z = self.current_positions[leg]
            
            # Math: Target - Current = Delta
            dx = target["dx"] - curr_x
            dy = target["dy"] - curr_y
            dz = target["dz"] - curr_z
            
            payload_deltas[leg] = [dx, dy, dz]
            
            # Update internal tracker so we know where the leg is now
            self.current_positions[leg] = [target["dx"], target["dy"], target["dz"]]

        self._send_to_robot(payload_deltas)

    def _send_to_robot(self, payload: dict):
        """Sends the calculated dx, dy, dz deltas to main_with_ik.py."""
        try:
            data = pickle.dumps(payload)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((SOCKET_HOST, SOCKET_PORT))
                s.sendall(data)
            print(f"✅ Sent deltas: {payload}")
        except ConnectionRefusedError:
            print("❌ Failed: Is main_with_ik.py running?")

# --- API Usage Simulation ---

if __name__ == "__main__":
    robot = QuadrupedStateManager()

    while True:
        command = input("\nEnter state (stand / sit) or 'q' to quit: ").strip().lower()
        
        if command == 'q':
            break
        elif command in ["stand", "sit"]:
            robot.change_state(command)
        else:
            print("Unknown command. Try 'stand' or 'sit'.")