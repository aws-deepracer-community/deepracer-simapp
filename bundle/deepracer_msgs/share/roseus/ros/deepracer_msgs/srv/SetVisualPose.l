;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualPose)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualPose (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALPOSE")
  (make-package "DEEPRACER_MSGS::SETVISUALPOSE"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALPOSEREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALPOSEREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALPOSERESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass deepracer_msgs::SetVisualPoseRequest
  :super ros::object
  :slots (_link_name _visual_name _pose _block ))

(defmethod deepracer_msgs::SetVisualPoseRequest
  (:init
   (&key
    ((:link_name __link_name) "")
    ((:visual_name __visual_name) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_name (string __link_name))
   (setq _visual_name (string __visual_name))
   (setq _pose __pose)
   (setq _block __block)
   self)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:visual_name
   (&optional __visual_name)
   (if __visual_name (setq _visual_name __visual_name)) _visual_name)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:block
   (&optional __block)
   (if __block (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; string _link_name
    4 (length _link_name)
    ;; string _visual_name
    4 (length _visual_name)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _link_name
       (write-long (length _link_name) s) (princ _link_name s)
     ;; string _visual_name
       (write-long (length _visual_name) s) (princ _visual_name s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _link_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _visual_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _visual_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualPoseResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod deepracer_msgs::SetVisualPoseResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualPose
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualPose :md5sum-) "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(setf (get deepracer_msgs::SetVisualPose :datatype-) "deepracer_msgs/SetVisualPose")
(setf (get deepracer_msgs::SetVisualPose :request) deepracer_msgs::SetVisualPoseRequest)
(setf (get deepracer_msgs::SetVisualPose :response) deepracer_msgs::SetVisualPoseResponse)

(defmethod deepracer_msgs::SetVisualPoseRequest
  (:response () (instance deepracer_msgs::SetVisualPoseResponse :init)))

(setf (get deepracer_msgs::SetVisualPoseRequest :md5sum-) "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(setf (get deepracer_msgs::SetVisualPoseRequest :datatype-) "deepracer_msgs/SetVisualPoseRequest")
(setf (get deepracer_msgs::SetVisualPoseRequest :definition-)
      "string link_name
string visual_name
geometry_msgs/Pose pose
bool block

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
string status_message
")

(setf (get deepracer_msgs::SetVisualPoseResponse :md5sum-) "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(setf (get deepracer_msgs::SetVisualPoseResponse :datatype-) "deepracer_msgs/SetVisualPoseResponse")
(setf (get deepracer_msgs::SetVisualPoseResponse :definition-)
      "string link_name
string visual_name
geometry_msgs/Pose pose
bool block

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
string status_message
")



(provide :deepracer_msgs/SetVisualPose "3b6fe41d1c80e9b62f81f01d6ed4fc64")


