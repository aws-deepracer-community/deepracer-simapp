;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualMesh)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualMesh (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALMESH")
  (make-package "DEEPRACER_MSGS::SETVISUALMESH"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALMESHREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALMESHREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALMESHRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALMESHRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass deepracer_msgs::SetVisualMeshRequest
  :super ros::object
  :slots (_link_name _visual_name _filename _scale _block ))

(defmethod deepracer_msgs::SetVisualMeshRequest
  (:init
   (&key
    ((:link_name __link_name) "")
    ((:visual_name __visual_name) "")
    ((:filename __filename) "")
    ((:scale __scale) (instance geometry_msgs::Vector3 :init))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_name (string __link_name))
   (setq _visual_name (string __visual_name))
   (setq _filename (string __filename))
   (setq _scale __scale)
   (setq _block __block)
   self)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:visual_name
   (&optional __visual_name)
   (if __visual_name (setq _visual_name __visual_name)) _visual_name)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:scale
   (&rest __scale)
   (if (keywordp (car __scale))
       (send* _scale __scale)
     (progn
       (if __scale (setq _scale (car __scale)))
       _scale)))
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
    ;; string _filename
    4 (length _filename)
    ;; geometry_msgs/Vector3 _scale
    (send _scale :serialization-length)
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
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;; geometry_msgs/Vector3 _scale
       (send _scale :serialize s)
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
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _scale
     (send _scale :deserialize buf ptr-) (incf ptr- (send _scale :serialization-length))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualMeshResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod deepracer_msgs::SetVisualMeshResponse
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

(defclass deepracer_msgs::SetVisualMesh
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualMesh :md5sum-) "f2c3b05d1c4b997abfc5202f04e76d70")
(setf (get deepracer_msgs::SetVisualMesh :datatype-) "deepracer_msgs/SetVisualMesh")
(setf (get deepracer_msgs::SetVisualMesh :request) deepracer_msgs::SetVisualMeshRequest)
(setf (get deepracer_msgs::SetVisualMesh :response) deepracer_msgs::SetVisualMeshResponse)

(defmethod deepracer_msgs::SetVisualMeshRequest
  (:response () (instance deepracer_msgs::SetVisualMeshResponse :init)))

(setf (get deepracer_msgs::SetVisualMeshRequest :md5sum-) "f2c3b05d1c4b997abfc5202f04e76d70")
(setf (get deepracer_msgs::SetVisualMeshRequest :datatype-) "deepracer_msgs/SetVisualMeshRequest")
(setf (get deepracer_msgs::SetVisualMeshRequest :definition-)
      "string link_name
string visual_name
string filename
geometry_msgs/Vector3 scale
bool block

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

")

(setf (get deepracer_msgs::SetVisualMeshResponse :md5sum-) "f2c3b05d1c4b997abfc5202f04e76d70")
(setf (get deepracer_msgs::SetVisualMeshResponse :datatype-) "deepracer_msgs/SetVisualMeshResponse")
(setf (get deepracer_msgs::SetVisualMeshResponse :definition-)
      "string link_name
string visual_name
string filename
geometry_msgs/Vector3 scale
bool block

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

")



(provide :deepracer_msgs/SetVisualMesh "f2c3b05d1c4b997abfc5202f04e76d70")


