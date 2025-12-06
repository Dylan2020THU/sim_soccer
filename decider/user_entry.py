#  user_entry.py
#       The entry file for your program
#
#   @description:
#       You have to implement two function:
#           1. def loop(agent) -> None
#           2. def init(agent) -> None
#       Just like the arduino phylosophy, init() is called once when 
#       the core is starting, and loop() is call every 0.1s (10Hz). 
#       'Agent' is the instance providing everything you need to interact
#       with the robot. See comment (and implement as well) on decider.py. 


def loop(agent) -> None:
    logger = agent.get_logger()
    ball_pos = agent.get_ball_pos()
    logger.info(f"ball_pos: {ball_pos}")
    pass


def init(agent) -> None:
    pass
