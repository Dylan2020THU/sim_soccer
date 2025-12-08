# Quick Guide to Agent Implementation

To help you quickly implement your own strategy or algorithm, this framework wraps all complex, platform-specific interfaces into the `Agent` class.

Your decision-making code should be developed in a structure similar to the Arduino IDE, requiring you to provide the following two functions in your `user_entry.py` file:

- **`def init(agent) -> None`**: This function is called once during system startup, after all initial components (ROS 2 node, interfaces) have been initialized and before the main loop begins. Use this for one-time setup and initialization.
    
- **`def loop(agent) -> None`**: This function is called continuously at a fixed frequency (typically 10Hz, or as defined by the `timer_period`). This is the main control loop for your strategy. This function **must not** block or wait under any circumstances.
    

The `agent` instance passed to these functions is the sole interface for interacting with the environment, encapsulating logic for sensor acquisition, self-localization, and motion control.

## Agent Interfaces

The `Agent` class provides a comprehensive set of methods categorized into Actuations (outputs) and Perception (inputs).

### Actuations

1.  **`cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float) -> None`**  
    Sets the robot's target **relative** velocity. The values are scaled and protected by configuration limits (e.g., `max_walk_vel_x`).
    
    - `vel_x`: Target velocity (m/s) pointing straight forward (positive) or backward (negative).
        
    - `vel_y`: Target velocity (m/s) pointing to the left (positive) or right (negative).
        
    - `vel_theta`: Target angular velocity (rad/s) for anti-clockwise spin (positive) or clockwise (negative).
        
2.  **`move_head(self, pitch: float, yaw: float) -> None`**  
    Sets the robot's head and neck angles, allowing the user to temporarily override the autonomous head control component.
    
    - `pitch`: The angle for tilting the head up/down.
        
    - `yaw`: The angle for shaking the head left/right.
        
    
    **Notice:** Call this method to take over head control. Set arguments to `math.nan` to return control to the dedicated `head_control` component.
    
3.  **`kick(self, foot=0, death=0) -> None`**  
    Executes a kicking action.
    
    - `foot`: `0` for left foot, `1` for right foot.
        
    - `death`: Set to `1` to execute a powerful kick that may cause the robot to fall over. **Use with caution.**
        
4.  **`save_ball(self, direction=1) -> None`**  
    Executes a goal-saving action.
    
    - `direction`: `1` for saving to the left, `2` for saving to the right.
    
    **Use with caution.**
    
5.  **`stop(self, sleep_time: float = 0) -> None`**  
    Stops all robot movement immediately.
    
    - `sleep_time`: An optional delay (in seconds) after the stop command is published, allowing for stability before the next command.

### Perception

#### Self-Localization & Configuration

1.  **`get_self_pos(self) -> np.ndarray`**  
    Returns the robot's current position (x, y) in the global map coordinates.
    
    - **Return Type:** `np.ndarray` (`[x, y]`)
2.  **`get_self_yaw(self) -> float`**  
    Returns the robot's current orientation (yaw) angle (rad) in map coordinates.
    
    - **Return Type:** `float`
3.  **`angle_normalize(self, angle: float) -> float`**  
    A utility function that normalizes any given angle (in radians) into the standard range of $(-\pi, \pi)$. Returns `None` if the input is `None`.
    
    - **Return Type:** `Optional[float]`
4.  **`get_config(self) -> Dict`**  
    Returns the agent's full configuration dictionary, typically loaded from a YAML or JSON file.
    
    - **Return Type:** `Dict`

#### Ball & Vision Data

1.  **`get_if_ball(self) -> bool`**  
    Returns `True` if the vision system currently detects the ball, `False` otherwise.
    
    - **Return Type:** `bool`
2.  **`get_ball_pos(self) -> List[Optional[float]]`**  
    Returns the ball's position relative to the robot's current location (`[x, y]` in meters), or `[None, None]` if the ball is not detected.
    
    - **Return Type:** `List[Optional[float]]` (`[x, y]`)
3.  **`get_ball_distance(self) -> float`**  
    Returns the distance (in meters) to the ball. Returns a large value (`1e6`) if the ball is not currently detected.
    
    - **Return Type:** `float`
4.  **`get_ball_angle(self) -> Optional[float]`**  
    Calculates and returns the angle (in radians) to the ball, relative to the robot's forward direction. Returns `None` if the ball is not detected.
    
    - **Return Type:** `Optional[float]`
5.  **`get_ball_pos_in_map(self) -> np.ndarray`**  
    Returns the ball's position (x, y) in the global map coordinates. Returns `None` if the ball is not detected.
    
    - **Return Type:** `np.ndarray` (`[x, y]`)
6.  **`get_if_close_to_ball(self) -> bool`**  
    Returns `True` if the distance to the ball is less than the configured `close_to_ball_threshold`.
    
    - **Return Type:** `bool`
7.  **`get_ball_history(self)`**  
    Returns the recent history of the ball's detected positions and associated metadata. The exact return type is defined by the underlying vision interface.
    
    - **Return Type:** `Any`
8.  **`get_neck(self) -> float`**  
    Returns the current angle (rad) of the robot's neck joint.
    
    - **Return Type:** `float`
9.  **`get_head(self) -> float`**  
    Returns the current angle (rad) of the robot's head joint.
    
    - **Return Type:** `float`


# Code structures

### Mandatory structures
There are two mandatroy directories due to the auto startup of systemd services: 
* `decider/scripts/*`

Which you should NEVER change permission, remove, rename, modify any files structures under this directory. 
And the Agent's dependencies:
* `interfaces/*` contains compatitable layer of underlying interfaces. 
* `configuration.py`  contains codes to parse config.yaml
* `config.yaml` is the file for configuration. Do not delete/modify, but feel free to append to.
* `config_override.yaml` is the file not tracking by git. Place your config here if it is robot-specific and shouldn't be tracked by git. Create it or not, this file is optional.
* `user_entry.py` is the entry file contains `loop(agent)` and `init(agent)`. Place your code here.

### Logging
We use ``logging`` compatitable logging functions. Example: 
```python
logger = agent.get_logger()
logger.debug(f"...") #debug, hide by default
logger.info(f"...") # info
logger.warning(f"...") # warn
logger.error(f"...") # error
```