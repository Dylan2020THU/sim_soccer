import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys

def plot_latest_log():
    log_dir = "debug_logs"
    list_of_files = glob.glob(os.path.join(log_dir, "*.csv"))
    if not list_of_files:
        print("No log files found in debug_logs/")
        return
        
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Plotting data from: {latest_file}")
    
    df = pd.read_csv(latest_file)
    
    # Filter out empty rows if any
    df = df.dropna()
    
    if len(df) == 0:
        print("Empty log file.")
        return

    # Create Plot
    fig, ax = plt.subplots(figsize=(10, 14))
    
    # 1. Plot Field Boundaries (M Field: 14x9)
    field_L = 14.0
    field_W = 9.0
    ax.plot([-field_W/2, field_W/2], [field_L/2, field_L/2], 'k-', linewidth=2) # Top (Goal)
    ax.plot([-field_W/2, field_W/2], [-field_L/2, -field_L/2], 'k-', linewidth=2) # Bottom
    ax.plot([-field_W/2, -field_W/2], [-field_L/2, field_L/2], 'k-', linewidth=2) # Left
    ax.plot([field_W/2, field_W/2], [-field_L/2, field_L/2], 'k-', linewidth=2) # Right
    
    # Goal
    ax.plot([-1.2, 1.2], [field_L/2, field_L/2], 'g-', linewidth=4, label='Goal')
    
    # Repulsion Lines (X = +/- 4.5 - 1.0 = +/- 3.5)
    ax.axvline(x=3.5, color='r', linestyle='--', alpha=0.3)
    ax.axvline(x=-3.5, color='r', linestyle='--', alpha=0.3)
    
    # 2. Plot Robot Trajectory
    sc = ax.scatter(df['rx'], df['ry'], c=df['time'], cmap='viridis', label='Robot Path', s=10)
    plt.colorbar(sc, label='Time')
    
    # 3. Plot Goal Vectors (Every 10th frame)
    step = 5
    quiver_df = df.iloc[::step]
    
    # Target Vector in Global Frame
    # t_vec_gx, t_vec_gy
    ax.quiver(quiver_df['rx'], quiver_df['ry'], 
              quiver_df['t_vec_gx'], quiver_df['t_vec_gy'], 
              color='blue', alpha=0.5, scale=20, width=0.003, label='Target Vec')
              
    # Command Vector (Local to Global)
    # Cmd is (cmd_x, cmd_y) in Robot Frame. Need to rotate by Robot Yaw to get Global.
    # Global X (Right), Y (Front). Robot Frame X (Front), Y (Left).
    # This is tricky. Let's trust the recorded global position changes instead?
    # Or just reconstruction:
    # vx_global = cmd_x * cos(yaw) - cmd_y * sin(yaw)
    # vy_global = cmd_x * sin(yaw) + cmd_y * cos(yaw)
    # (Assuming standard math definition: Yaw 0 = X+, Yaw 90 = Y+)
    # (And Cmd X = Forward, Cmd Y = Left... Wait, usually Y is Left means +90 deg relative to X)
    # So v_body = (cmd_x, cmd_y). 
    # v_global = R(yaw) * v_body?
    # If Body X is Forward, and Body Y is Left.
    # v_global_x = cmd_x * cos(yaw) - cmd_y * sin(yaw)
    # v_global_y = cmd_x * sin(yaw) + cmd_y * cos(yaw)
    
    import numpy as np
    yaw_rad = np.radians(quiver_df['ryaw'])
    cmd_x = quiver_df['cmd_x']
    cmd_y = quiver_df['cmd_y']
    
    c = np.cos(yaw_rad)
    s = np.sin(yaw_rad)
    
    # Assuming MOS definition: Yaw 0 = Right, Yaw 90 = Front.
    # Robot cmd_x is Forward (aligned with Yaw).
    # Robot cmd_y is Left (aligned with Yaw + 90).
    
    # Global components of Forward vector (unit): (cos(yaw), sin(yaw))
    # Global components of Left vector (unit):    (-sin(yaw), cos(yaw))
    
    v_gx = cmd_x * c + cmd_y * (-s)
    v_gy = cmd_x * s + cmd_y * (c)
    
    ax.quiver(quiver_df['rx'], quiver_df['ry'], 
              v_gx, v_gy, 
              color='red', alpha=0.5, scale=20, width=0.003, label='Cmd Vel')

    ax.set_aspect('equal')
    ax.set_title("Dribble Debug Trajectory")
    ax.set_xlabel("X (Width)")
    ax.set_ylabel("Y (Length)")
    ax.legend()
    ax.grid(True)
    
    output_path = latest_file.replace('.csv', '.png')
    plt.savefig(output_path)
    print(f"Plot saved to: {output_path}")

if __name__ == "__main__":
    plot_latest_log()
