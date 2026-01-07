import sys
import os
import time
import json
import logging

# Helper to find zmq_wrapper
# mos-brain/decider/interfaces -> mos-brain/simulation/soccerlab_bridge/common
COMMON_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../simulation/soccerlab_bridge/common"))
if COMMON_PATH not in sys.path:
    sys.path.append(COMMON_PATH)

try:
    from zmq_wrapper import ZMQWrapper
    import zmq
except ImportError:
    print(f"[SimClient] Error: Could not import zmq_wrapper or zmq. Ensure pyzmq is installed and path is correct: {COMMON_PATH}")
    # Fallback or exit
    ZMQWrapper = None

class SimClient:
    def __init__(self, ip="127.0.0.1", port="5555"):
        self.logger = logging.getLogger("SimClient")
        
        if ZMQWrapper is None:
            raise RuntimeError("ZMQ dependencies missing.")

        self.ip = ip
        self.port = port
        self.addr = f"tcp://{ip}:{port}"
        
        self.client = ZMQWrapper(socket_type=zmq.REQ, addr=self.addr)
        self.client.connect()
        self.logger.info(f"Connected to Sim Server at {self.addr}")
        
    def communicate(self, cmd_vel=[0.0, 0.0, 0.0]):
        """
        Send command and wait for state.
        cmd_vel: [vx, vy, w]
        Returns: proper state dict or None on failure
        """
        req = {
            "cmd": cmd_vel,
            "timestamp": time.time()
        }
        
        try:
            self.client.send_json(req)
            resp = self.client.recv_json()
            # Basic validation
            if not resp or "state" not in resp:
                self.logger.warning("Received invalid response from Sim.")
                return None
            return resp
        except Exception as e:
            self.logger.error(f"Communication error: {e}")
            return None

    def close(self):
        if self.client:
            self.client.close()
