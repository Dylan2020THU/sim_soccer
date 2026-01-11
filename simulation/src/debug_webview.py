
import sys
import os
import time
import threading
import requests
import types

# Mock modules for Gym/IsaacLab (since we only want to test Flask/Webview)
m = types.ModuleType('gym')
m.Wrapper = object
sys.modules['gym'] = m

m2 = types.ModuleType('isaaclab')
m2.envs = types.ModuleType('isaaclab.envs')
m2.envs.ui = types.ModuleType('isaaclab.envs.ui')
# Use a dummy class for ViewportCameraController
class DummyController:
    pass
m2.envs.ui.ViewportCameraController = DummyController
sys.modules['isaaclab'] = m2
sys.modules['isaaclab.envs'] = m2.envs
sys.modules['isaaclab.envs.ui'] = m2.envs.ui

# Replicate sim_server.py path setup
# This file is in mos-brain/simulation/soccerlab_bridge/debug_webview.py
# So __file__ is .../soccerlab_bridge/debug_webview.py
mos_brain_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ws_root = os.path.dirname(mos_brain_root)
labwebview_path = os.environ.get("LABWEBVIEW_PATH", os.path.join(ws_root, "labWebView"))

print(f"WS Root: {ws_root}")
print(f"LabWebView Path: {labwebview_path}")

if os.path.exists(labwebview_path):
    # USE THE FIX: Insert at 0
    sys.path.insert(0, os.path.join(labwebview_path, "source", "labwebView"))
    print(f"Added {os.path.join(labwebview_path, 'source', 'labwebView')} to sys.path at index 0")
else:
    print("LabWebView path not found!")
    sys.exit(1)

try:
    import labWebView
    print(f"Imported labWebView from: {labWebView.__file__}")
    print(f"LABWEBVIEW_TEMPLATE_DIR: {labWebView.LABWEBVIEW_TEMPLATE_DIR}")
    
    # Check index.html
    index_path = os.path.join(labWebView.LABWEBVIEW_TEMPLATE_DIR, "index.html")
    if os.path.exists(index_path):
        print(f"Found index.html at: {index_path}")
    else:
        print(f"ERROR: index.html NOT found at: {index_path}")
        
    # Try importing wrapper and app (NOW WITH REAL FLASK INSTALLED)
    from labWebView.wrapper import app, socketio
    print("Imported app from wrapper successfully!")
    
    # Run server in thread
    def run_server():
        try:
            # Match wrapper.py run call
            print("Starting socketio run on port 5899...")
            socketio.run(app, host='127.0.0.1', port=5899, use_reloader=False, debug=False) 
        except Exception as e:
            print(f"Server error: {e}")

    t = threading.Thread(target=run_server)
    t.daemon = True
    t.start()
    print("Server thread started. Waiting 3s...")
    time.sleep(3) # Wait for startup
    
    # Request
    try:
        url = "http://127.0.0.1:5899/"
        print(f"Requesting {url}...")
        r = requests.get(url, timeout=5)
        print(f"Request to / returned status: {r.status_code}")
        if r.status_code == 200:
            print("Success! Content length:", len(r.text))
        else:
            print("Failed! Content:", r.text[:200])
    except Exception as e:
        print(f"Request failed: {e}")
        
except ImportError as e:
    print(f"Import failed: {e}")
    import traceback
    traceback.print_exc()
except Exception as e:
    print(f"Unexpected error: {e}")
    import traceback
    traceback.print_exc()
