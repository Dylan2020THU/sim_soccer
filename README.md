# A quick guide to implement your thoughts 

To help quickly implement your own algorithm of strategy, we make this framework, wrapping all complex and platform related interface into a class "Agent". Your own code should be develop in the manner of Arduino IDE, that is, you should provide these two functions :
* `def loop(agent) -> None`, which is going to be call at a frequence (typicall 10Hz). This function **shouldn't** block or wait under any circumstances. 
* `def init(agent) -> None`, which is going to be call once before the loop start but after some initiazing work finished.

The pointer (or, take it as an instance, if you're not interesting in optimizing) `agent` is all you need and all you should rely on to the interact environment. Agent provide an abstract of all other tasks, e.g acquire the location of ball, knowing where the robot is, sending command to motion controller. 

# Interfaces in Agent
### Actuations
* `def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float) -> None:`
	-  Set robot velocity with scaling from configuration.
	-  vel_x: the target **relative** velocity pointing straight forward.
	-  vel_y: the target **relative** velocity pointing to the left.
	-  vel_theta: the target **relative** angular velocity spinning anti-clockwise. 

* `def move_head(self, pitch: float, yaw: float) -> None:`
	- Set robot's head and neck angles.
	- pitch: the angle of bending the head.
	- yaw: the angle of shaking the head.
	> Notice: At most time, head is controlled by another component "head_control". Call this to take over, and set to NaN to cancel takeover.

* `def kick(self, foot=0, death=0) -> None:`
	- Set robot's head and neck angles.
	- foot: which foot to kick. 0 to the left and 1 to the right.
	- death: whether to kick hard (and fall down)
	> Dangerous !

* `def save(self, direction) -> None:`
	- Execute the saving action.
	- direction: which side. 1 to the left and 2 to the right.
	> Dangerous !

* `def stop(self, sleep_time: float = 0) -> None:`
	- Stop the robot's movement and optionally sleep.
	-  sleep_time: waiting time (in seconds) after publishing the command.

### Perception


To be continue.