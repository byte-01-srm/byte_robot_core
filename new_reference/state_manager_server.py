import pickle
import socket
import asyncio
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER, MAX_LIVE_DEG_PER_S
from lib.trot_gait import TrotGaitController
import time
import copy

# --- Data Models ---

class StateRequest(BaseModel):
    new_state: str
    sender_id: str

# --- Core Logic ---

class QuadrupedStateManager:
    def __init__(self):
        # Internal tracking of the current position
        self.current_positions = {leg: [0.0, 0.0, 0.0] for leg in LEG_ORDER}
        
        # Define Coordinate targets
        self.static_states = {
            "sit":   {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "stand": {"dx": 0.0, "dy": 0.0, "dz": -14.0},
        }

        self.dynamic_states = {
            "walk":  {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "climb": {"dx": 0.0, "dy": 0.0, "dz": 0.0},
        }
        
        # The Lock ensures only one movement happens at a time
        self.lock = asyncio.Lock()

    # ─────────────────────────────────────────────────────────────────────────────
    # CORE TROT GAIT RUNNER
    # ─────────────────────────────────────────────────────────────────────────────
    async def _run_trot_gait_cycle(self, axis: str, direction: int):
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

        # 1. CREATE A DEEP COPY (Safe from the shallow copy trap)
        positions_before_trot = copy.deepcopy(self.current_positions)
        
        # create a controller instance for each leg and calculate foot positions
        controller = {}  # reset controller dict each loop to ensure clean state
        for leg in LEG_ORDER:
            controller[leg] = TrotGaitController(leg, axis, direction)

        last_curve_pos = {leg: controller[leg]._foot_pos_phase(0.0) for leg in LEG_ORDER}

        while phase < 1.0: 
            payload_deltas = {}

            for leg in LEG_ORDER: 
                # 1. Get the ABSOLUTE position on the gait curve for this exact tick
                curve_x, curve_y, curve_z = controller[leg]._foot_pos_phase(phase)
                
                # 2. Convert absolute curve position into a RELATIVE delta for the main script
                dx = curve_x - last_curve_pos[leg][0]
                dy = curve_y - last_curve_pos[leg][1]
                dz = curve_z - last_curve_pos[leg][2]

                # 3. Update the curve tracker for the next loop
                last_curve_pos[leg] = [curve_x, curve_y, curve_z]

                # 4. Send the true delta directly to the physical robot
                payload_deltas[leg] = [dx, dy, dz]

                # 5. Update the API's GLOBAL tracker by adding the delta
                self.current_positions[leg][0] += dx
                self.current_positions[leg][1] += dy
                self.current_positions[leg][2] += dz 

            # Send the command to the physical robot hardware
            success = await self._send_to_robot_async(payload_deltas, speed=GAIT_SPEED_DEG_PER_S)
            print("\n")
            # time.sleep(dt)
            await asyncio.sleep(dt)
            phase += phase_step

        # 2. CALCULATE AND SEND THE PHYSICAL HARDWARE SNAP (From the previous fix)
        snap_deltas = {}
        for leg in LEG_ORDER:
            err_x = positions_before_trot[leg][0] - self.current_positions[leg][0]
            err_y = positions_before_trot[leg][1] - self.current_positions[leg][1]
            err_z = positions_before_trot[leg][2] - self.current_positions[leg][2]
            snap_deltas[leg] = [err_x, err_y, err_z]
            
        await self._send_to_robot_async(snap_deltas)
        
        # 3. RESET THE SOFTWARE TRACKER
        self.current_positions = copy.deepcopy(positions_before_trot)
        
        # 2. CALCULATE AND SEND THE PHYSICAL HARDWARE SNAP
        # snap_deltas = {}
        # for leg in LEG_ORDER:
        #     err_x = positions_before_trot[leg][0] - self.current_positions[leg][0]
        #     err_y = positions_before_trot[leg][1] - self.current_positions[leg][1]
        #     err_z = positions_before_trot[leg][2] - self.current_positions[leg][2]
        #     snap_deltas[leg] = [err_x, err_y, err_z]
            
        # print(f"\n📐 Trot cycle complete. Sending snap correction to robot.. {snap_deltas}")
        # await self._send_to_robot_async(snap_deltas)
        
        # # 3. RESET THE SOFTWARE TRACKER (Your original idea)
        # self.current_positions = copy.deepcopy(positions_before_trot)
        
        phase = 0.0

    async def change_state(self, target_state: str, sender: str):
        """API-like function to trigger a state transition with concurrency protection."""
        
        # 1. IMMEDIATE CHECK: If the robot is already moving, don't queue. Reject.
        if self.lock.locked():
            raise HTTPException(
                status_code=429, 
                detail=f"Robot is busy. Request from {sender} rejected."
            )

        async with self.lock:
            if target_state not in self.static_states and target_state not in self.dynamic_states:
                raise HTTPException(status_code=400, detail=f"State '{target_state}' unknown.")

            if target_state in self.static_states:
                target = self.static_states[target_state]
                payload_deltas = {}
                
                # Calculate math inside the lock to ensure we use the latest current_positions
                for leg in LEG_ORDER:
                    curr_x, curr_y, curr_z = self.current_positions[leg]
                    
                    dx = target["dx"] - curr_x
                    dy = target["dy"] - curr_y
                    dz = target["dz"] - curr_z
                    
                    payload_deltas[leg] = [dx, dy, dz]
                    
                    # Update internal tracker immediately
                    self.current_positions[leg] = [target["dx"], target["dy"], target["dz"]]

                # Send the command to the physical robot hardware
                success = await self._send_to_robot_async(payload_deltas)
                
                if not success:
                    raise HTTPException(status_code=503, detail="Robot hardware communication failed.")

                # 2. SIMULATE MOVEMENT TIME: 
                # We hold the lock for 2 seconds so no other state can be triggered 
                # while the legs are physically moving.
                await asyncio.sleep(2.0)
                
                return {"status": "success", "reached": target_state}
            else:
                if target_state == "walk":
                    # For dynamic states, we can call a separate function that handles the gait cycle
                    await self._run_trot_gait_cycle(axis='y', direction=1)
                
                return {"status": "success", "started": target_state}

    # def _send_to_robot(self, payload: dict):
    #     """Standard socket communication (Synchronous)."""
    #     try:
    #         data = pickle.dumps(payload)
    #         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #             s.settimeout(1.0)
    #             s.connect((SOCKET_HOST, SOCKET_PORT))
    #             s.sendall(data)
    #         return True
    #     except Exception as e:
    #         print(f"Socket Error: {e}")
    #         return False

    async def _send_to_robot_async(self, payload: dict, speed: float = MAX_LIVE_DEG_PER_S):
        try:
            payload["speed"] = speed  # Add speed to the payload for the robot to use
            data = pickle.dumps(payload)
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(SOCKET_HOST, SOCKET_PORT), 
                timeout=1.0
            )
            writer.write(data)
            await writer.drain()
            writer.close()
            await writer.wait_closed()
            return True
        except Exception as e:
            print(f"Socket Error: {e}")
            return False

# --- FastAPI App ---

app = FastAPI(title="Quadruped API")
robot_manager = QuadrupedStateManager()

@app.post("/change-state")
async def handle_state_change(request: StateRequest):
    # This calls our manager which handles the locking logic
    result = await robot_manager.change_state(request.new_state, request.sender_id)
    
    return {
        "sender_id": request.sender_id,
        "requested_state": request.new_state,
        "result": result
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)