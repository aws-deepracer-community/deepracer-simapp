;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetModelStates)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetModelStates (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETMODELSTATES")
  (make-package "DEEPRACER_MSGS::SETMODELSTATES"))
(unless (find-package "DEEPRACER_MSGS::SETMODELSTATESREQUEST")
  (make-package "DEEPRACER_MSGS::SETMODELSTATESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETMODELSTATESRESPONSE")
  (make-package "DEEPRACER_MSGS::SETMODELSTATESRESPONSE"))

(in-package "ROS")

(if (not (find-package "GAZEBO_MSGS"))
  (ros::roseus-add-msgs "gazebo_msgs"))




(defclass deepracer_msgs::SetModelStatesRequest
  :super ros::object
  :slots (_model_states ))

(defmethod deepracer_msgs::SetModelStatesRequest
  (:init
   (&key
    ((:model_states __model_states) (let (r) (dotimes (i 0) (push (instance gazebo_msgs::ModelState :init) r)) r))
    )
   (send-super :init)
   (setq _model_states __model_states)
   self)
  (:model_states
   (&rest __model_states)
   (if (keywordp (car __model_states))
       (send* _model_states __model_states)
     (progn
       (if __model_states (setq _model_states (car __model_states)))
       _model_states)))
  (:serialization-length
   ()
   (+
    ;; gazebo_msgs/ModelState[] _model_states
    (apply #'+ (send-all _model_states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; gazebo_msgs/ModelState[] _model_states
     (write-long (length _model_states) s)
     (dolist (elem _model_states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; gazebo_msgs/ModelState[] _model_states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _model_states (let (r) (dotimes (i n) (push (instance gazebo_msgs::ModelState :init) r)) r))
     (dolist (elem- _model_states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass deepracer_msgs::SetModelStatesResponse
  :super ros::object
  :slots (_success _status_message _status _messages ))

(defmethod deepracer_msgs::SetModelStatesResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    ((:messages __messages) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   (setq _status __status)
   (setq _messages __messages)
   self)
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

(defclass deepracer_msgs::SetModelStates
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetModelStates :md5sum-) "302646b3654009f39a95e719d97c8571")
(setf (get deepracer_msgs::SetModelStates :datatype-) "deepracer_msgs/SetModelStates")
(setf (get deepracer_msgs::SetModelStates :request) deepracer_msgs::SetModelStatesRequest)
(setf (get deepracer_msgs::SetModelStates :response) deepracer_msgs::SetModelStatesResponse)

(defmethod deepracer_msgs::SetModelStatesRequest
  (:response () (instance deepracer_msgs::SetModelStatesResponse :init)))

(setf (get deepracer_msgs::SetModelStatesRequest :md5sum-) "302646b3654009f39a95e719d97c8571")
(setf (get deepracer_msgs::SetModelStatesRequest :datatype-) "deepracer_msgs/SetModelStatesRequest")
(setf (get deepracer_msgs::SetModelStatesRequest :definition-)
      "gazebo_msgs/ModelState[] model_states

================================================================================
MSG: gazebo_msgs/ModelState
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
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
---
bool success
string status_message
int8[] status
string[] messages
")

(setf (get deepracer_msgs::SetModelStatesResponse :md5sum-) "302646b3654009f39a95e719d97c8571")
(setf (get deepracer_msgs::SetModelStatesResponse :datatype-) "deepracer_msgs/SetModelStatesResponse")
(setf (get deepracer_msgs::SetModelStatesResponse :definition-)
      "gazebo_msgs/ModelState[] model_states

================================================================================
MSG: gazebo_msgs/ModelState
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
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
---
bool success
string status_message
int8[] status
string[] messages
")



(provide :deepracer_msgs/SetModelStates "302646b3654009f39a95e719d97c8571")


