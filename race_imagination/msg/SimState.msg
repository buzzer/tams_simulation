# Define the simulation state to create a simulation and execute an action
# Copied from race_imagination/Imagine.action
#
Header header
string action # move_base, pickup, place etc.
race_msgs/ParamList[] parameter # [north, south] [slow, normal, fast] [peppermill]
race_msgs/WorldState world_state
race_msgs/RobotState robot_state
float32 confidence # 0 .. 1.0 confidence value for the simulated action to succeed
