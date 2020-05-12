;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::GetLinkStates)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'GetLinkStates (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::GETLINKSTATES")
  (make-package "DEEPRACER_MSGS::GETLINKSTATES"))
(unless (find-package "DEEPRACER_MSGS::GETLINKSTATESREQUEST")
  (make-package "DEEPRACER_MSGS::GETLINKSTATESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::GETLINKSTATESRESPONSE")
  (make-package "DEEPRACER_MSGS::GETLINKSTATESRESPONSE"))

(in-package "ROS")



(if (not (find-package "GAZEBO_MSGS"))
  (ros::roseus-add-msgs "gazebo_msgs"))


(defclass deepracer_msgs::GetLinkStatesRequest
  :super ros::object
  :slots (_link_names _reference_frames ))

(defmethod deepracer_msgs::GetLinkStatesRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:reference_frames __reference_frames) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _reference_frames __reference_frames)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:reference_frames
   (&optional __reference_frames)
   (if __reference_frames (setq _reference_frames __reference_frames)) _reference_frames)
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; string[] _reference_frames
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _reference_frames)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _link_names
     (write-long (length _link_names) s)
     (dolist (elem _link_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _reference_frames
     (write-long (length _reference_frames) s)
     (dolist (elem _reference_frames)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _link_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _link_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _reference_frames
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reference_frames (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _reference_frames i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass deepracer_msgs::GetLinkStatesResponse
  :super ros::object
  :slots (_link_states _success _status_message _status _messages ))

(defmethod deepracer_msgs::GetLinkStatesResponse
  (:init
   (&key
    ((:link_states __link_states) (let (r) (dotimes (i 0) (push (instance gazebo_msgs::LinkState :init) r)) r))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    ((:messages __messages) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _link_states __link_states)
   (setq _success __success)
   (setq _status_message (string __status_message))
   (setq _status __status)
   (setq _messages __messages)
   self)
  (:link_states
   (&rest __link_states)
   (if (keywordp (car __link_states))
       (send* _link_states __link_states)
     (progn
       (if __link_states (setq _link_states (car __link_states)))
       _link_states)))
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:messages
   (&optional __messages)
   (if __messages (setq _messages __messages)) _messages)
  (:serialization-length
   ()
   (+
    ;; gazebo_msgs/LinkState[] _link_states
    (apply #'+ (send-all _link_states :serialization-length)) 4
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ;; int8[] _status
    (* 1    (length _status)) 4
    ;; string[] _messages
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _messages)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; gazebo_msgs/LinkState[] _link_states
     (write-long (length _link_states) s)
     (dolist (elem _link_states)
       (send elem :serialize s)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;; int8[] _status
     (write-long (length _status) s)
     (dotimes (i (length _status))
       (write-byte (elt _status i) s)
       )
     ;; string[] _messages
     (write-long (length _messages) s)
     (dolist (elem _messages)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; gazebo_msgs/LinkState[] _link_states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_states (let (r) (dotimes (i n) (push (instance gazebo_msgs::LinkState :init) r)) r))
     (dolist (elem- _link_states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8[] _status
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _status (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _status i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _status i) 127) (setf (elt _status i) (- (elt _status i) 256)))
     ))
   ;; string[] _messages
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _messages (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _messages i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass deepracer_msgs::GetLinkStates
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::GetLinkStates :md5sum-) "1bc3add906c1ea529af2b9b4ae208741")
(setf (get deepracer_msgs::GetLinkStates :datatype-) "deepracer_msgs/GetLinkStates")
(setf (get deepracer_msgs::GetLinkStates :request) deepracer_msgs::GetLinkStatesRequest)
(setf (get deepracer_msgs::GetLinkStates :response) deepracer_msgs::GetLinkStatesResponse)

(defmethod deepracer_msgs::GetLinkStatesRequest
  (:response () (instance deepracer_msgs::GetLinkStatesResponse :init)))

(setf (get deepracer_msgs::GetLinkStatesRequest :md5sum-) "1bc3add906c1ea529af2b9b4ae208741")
(setf (get deepracer_msgs::GetLinkStatesRequest :datatype-) "deepracer_msgs/GetLinkStatesRequest")
(setf (get deepracer_msgs::GetLinkStatesRequest :definition-)
      "string[] link_names

string[] reference_frames


---
gazebo_msgs/LinkState[] link_states
bool success
string status_message
int8[] status
string[] messages

================================================================================
MSG: gazebo_msgs/LinkState
# @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.
string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this link/body
                            # leave empty or \"world\" or \"map\" defaults to world-frame

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")

(setf (get deepracer_msgs::GetLinkStatesResponse :md5sum-) "1bc3add906c1ea529af2b9b4ae208741")
(setf (get deepracer_msgs::GetLinkStatesResponse :datatype-) "deepracer_msgs/GetLinkStatesResponse")
(setf (get deepracer_msgs::GetLinkStatesResponse :definition-)
      "string[] link_names

string[] reference_frames


---
gazebo_msgs/LinkState[] link_states
bool success
string status_message
int8[] status
string[] messages

================================================================================
MSG: gazebo_msgs/LinkState
# @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.
string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this link/body
                            # leave empty or \"world\" or \"map\" defaults to world-frame

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
")



(provide :deepracer_msgs/GetLinkStates "1bc3add906c1ea529af2b9b4ae208741")


