# Custom ModelState message for DeepRacer migration
# Replaces gazebo_msgs/ModelState for ROS 2 compatibility

string model_name
geometry_msgs/Pose pose
geometry_msgs/Twist twist
string reference_frame
