# Message to publish base translational velocity, acceleration and jerk
# angular velocity, acceleration, jerk
std_msgs/Header header
uint32 EpisodeID 
string state # "notopple", "semitopple", "topple"
geometry_msgs/Vector3Stamped obj_dev
race_pr2_markers/BaseDynamics obj_dyn
race_pr2_markers/BaseDynamics base_dyn # dynamics from the localization device
race_pr2_markers/BaseDynamics base_ctl # dynamics sent to base controller

