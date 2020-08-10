;; Auto-generated. Do not edit!


(when (boundp 'deepracer_simulation_environment::AgentRewardData)
  (if (not (find-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
    (make-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
  (shadow 'AgentRewardData (find-package "DEEPRACER_SIMULATION_ENVIRONMENT")))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::AGENTREWARDDATA")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::AGENTREWARDDATA"))

(in-package "ROS")
;;//! \htmlinclude AgentRewardData.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass deepracer_simulation_environment::AgentRewardData
  :super ros::object
  :slots (_agent_name _action _reward _action_space_len _speed_list _steering_angle_list _image ))

(defmethod deepracer_simulation_environment::AgentRewardData
  (:init
   (&key
    ((:agent_name __agent_name) "")
    ((:action __action) 0)
    ((:reward __reward) 0.0)
    ((:action_space_len __action_space_len) 0)
    ((:speed_list __speed_list) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:steering_angle_list __steering_angle_list) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _agent_name (string __agent_name))
   (setq _action (round __action))
   (setq _reward (float __reward))
   (setq _action_space_len (round __action_space_len))
   (setq _speed_list __speed_list)
   (setq _steering_angle_list __steering_angle_list)
   (setq _image __image)
   self)
  (:agent_name
   (&optional __agent_name)
   (if __agent_name (setq _agent_name __agent_name)) _agent_name)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:reward
   (&optional __reward)
   (if __reward (setq _reward __reward)) _reward)
  (:action_space_len
   (&optional __action_space_len)
   (if __action_space_len (setq _action_space_len __action_space_len)) _action_space_len)
  (:speed_list
   (&optional __speed_list)
   (if __speed_list (setq _speed_list __speed_list)) _speed_list)
  (:steering_angle_list
   (&optional __steering_angle_list)
   (if __steering_angle_list (setq _steering_angle_list __steering_angle_list)) _steering_angle_list)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; string _agent_name
    4 (length _agent_name)
    ;; int8 _action
    1
    ;; float64 _reward
    8
    ;; int8 _action_space_len
    1
    ;; string[] _speed_list
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _speed_list)) 4
    ;; string[] _steering_angle_list
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _steering_angle_list)) 4
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _agent_name
       (write-long (length _agent_name) s) (princ _agent_name s)
     ;; int8 _action
       (write-byte _action s)
     ;; float64 _reward
       (sys::poke _reward (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _action_space_len
       (write-byte _action_space_len s)
     ;; string[] _speed_list
     (write-long (length _speed_list) s)
     (dolist (elem _speed_list)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _steering_angle_list
     (write-long (length _steering_angle_list) s)
     (dolist (elem _steering_angle_list)
       (write-long (length elem) s) (princ elem s)
       )
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _agent_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _agent_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _action
     (setq _action (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _action 127) (setq _action (- _action 256)))
   ;; float64 _reward
     (setq _reward (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _action_space_len
     (setq _action_space_len (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _action_space_len 127) (setq _action_space_len (- _action_space_len 256)))
   ;; string[] _speed_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speed_list (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _speed_list i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _steering_angle_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _steering_angle_list (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _steering_angle_list i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(setf (get deepracer_simulation_environment::AgentRewardData :md5sum-) "af5e8658bf60dab3f19408bd4ad157dd")
(setf (get deepracer_simulation_environment::AgentRewardData :datatype-) "deepracer_simulation_environment/AgentRewardData")
(setf (get deepracer_simulation_environment::AgentRewardData :definition-)
      "string agent_name
int8 action
float64 reward
int8 action_space_len
string[] speed_list
string[] steering_angle_list
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :deepracer_simulation_environment/AgentRewardData "af5e8658bf60dab3f19408bd4ad157dd")


