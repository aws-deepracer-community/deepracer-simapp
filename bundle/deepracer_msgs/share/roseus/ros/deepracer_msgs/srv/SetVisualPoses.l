;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualPoses)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualPoses (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALPOSES")
  (make-package "DEEPRACER_MSGS::SETVISUALPOSES"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALPOSESREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALPOSESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALPOSESRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALPOSESRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass deepracer_msgs::SetVisualPosesRequest
  :super ros::object
  :slots (_link_names _visual_names _poses _block ))

(defmethod deepracer_msgs::SetVisualPosesRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:poses __poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   (setq _poses __poses)
   (setq _block __block)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:block
   (&optional __block)
   (if __block (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; string[] _visual_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _visual_names)) 4
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ;; bool _block
    1
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
     ;; string[] _visual_names
     (write-long (length _visual_names) s)
     (dolist (elem _visual_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
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
   ;; string[] _visual_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _visual_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _visual_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualPosesResponse
  :super ros::object
  :slots (_success _status_message _status _messages ))

(defmethod deepracer_msgs::SetVisualPosesResponse
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

(defclass deepracer_msgs::SetVisualPoses
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualPoses :md5sum-) "d316ff8d78498f26ce79e9a43ac6ed65")
(setf (get deepracer_msgs::SetVisualPoses :datatype-) "deepracer_msgs/SetVisualPoses")
(setf (get deepracer_msgs::SetVisualPoses :request) deepracer_msgs::SetVisualPosesRequest)
(setf (get deepracer_msgs::SetVisualPoses :response) deepracer_msgs::SetVisualPosesResponse)

(defmethod deepracer_msgs::SetVisualPosesRequest
  (:response () (instance deepracer_msgs::SetVisualPosesResponse :init)))

(setf (get deepracer_msgs::SetVisualPosesRequest :md5sum-) "d316ff8d78498f26ce79e9a43ac6ed65")
(setf (get deepracer_msgs::SetVisualPosesRequest :datatype-) "deepracer_msgs/SetVisualPosesRequest")
(setf (get deepracer_msgs::SetVisualPosesRequest :definition-)
      "string[] link_names
string[] visual_names
geometry_msgs/Pose[] poses
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
int8[] status
string[] messages
")

(setf (get deepracer_msgs::SetVisualPosesResponse :md5sum-) "d316ff8d78498f26ce79e9a43ac6ed65")
(setf (get deepracer_msgs::SetVisualPosesResponse :datatype-) "deepracer_msgs/SetVisualPosesResponse")
(setf (get deepracer_msgs::SetVisualPosesResponse :definition-)
      "string[] link_names
string[] visual_names
geometry_msgs/Pose[] poses
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
int8[] status
string[] messages
")



(provide :deepracer_msgs/SetVisualPoses "d316ff8d78498f26ce79e9a43ac6ed65")


