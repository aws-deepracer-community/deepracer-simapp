;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::GetVisuals)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'GetVisuals (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::GETVISUALS")
  (make-package "DEEPRACER_MSGS::GETVISUALS"))
(unless (find-package "DEEPRACER_MSGS::GETVISUALSREQUEST")
  (make-package "DEEPRACER_MSGS::GETVISUALSREQUEST"))
(unless (find-package "DEEPRACER_MSGS::GETVISUALSRESPONSE")
  (make-package "DEEPRACER_MSGS::GETVISUALSRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass deepracer_msgs::GetVisualsRequest
  :super ros::object
  :slots (_link_names _visual_names ))

(defmethod deepracer_msgs::GetVisualsRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; string[] _visual_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _visual_names)) 4
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
   ;;
   self)
  )

(defclass deepracer_msgs::GetVisualsResponse
  :super ros::object
  :slots (_link_names _visual_names _ambients _diffuses _speculars _emissives _transparencies _visibles _geometry_types _mesh_geom_filenames _mesh_geom_scales _poses _success _status_message _status _messages ))

(defmethod deepracer_msgs::GetVisualsResponse
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:ambients __ambients) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:diffuses __diffuses) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:speculars __speculars) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:emissives __emissives) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:transparencies __transparencies) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:visibles __visibles) (make-array 0 :initial-element 0 :element-type :integer))
    ((:geometry_types __geometry_types) (make-array 0 :initial-element 0 :element-type :integer))
    ((:mesh_geom_filenames __mesh_geom_filenames) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:mesh_geom_scales __mesh_geom_scales) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Vector3 :init) r)) r))
    ((:poses __poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    ((:messages __messages) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   (setq _ambients __ambients)
   (setq _diffuses __diffuses)
   (setq _speculars __speculars)
   (setq _emissives __emissives)
   (setq _transparencies __transparencies)
   (setq _visibles __visibles)
   (setq _geometry_types __geometry_types)
   (setq _mesh_geom_filenames __mesh_geom_filenames)
   (setq _mesh_geom_scales __mesh_geom_scales)
   (setq _poses __poses)
   (setq _success __success)
   (setq _status_message (string __status_message))
   (setq _status __status)
   (setq _messages __messages)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:ambients
   (&rest __ambients)
   (if (keywordp (car __ambients))
       (send* _ambients __ambients)
     (progn
       (if __ambients (setq _ambients (car __ambients)))
       _ambients)))
  (:diffuses
   (&rest __diffuses)
   (if (keywordp (car __diffuses))
       (send* _diffuses __diffuses)
     (progn
       (if __diffuses (setq _diffuses (car __diffuses)))
       _diffuses)))
  (:speculars
   (&rest __speculars)
   (if (keywordp (car __speculars))
       (send* _speculars __speculars)
     (progn
       (if __speculars (setq _speculars (car __speculars)))
       _speculars)))
  (:emissives
   (&rest __emissives)
   (if (keywordp (car __emissives))
       (send* _emissives __emissives)
     (progn
       (if __emissives (setq _emissives (car __emissives)))
       _emissives)))
  (:transparencies
   (&optional __transparencies)
   (if __transparencies (setq _transparencies __transparencies)) _transparencies)
  (:visibles
   (&optional __visibles)
   (if __visibles (setq _visibles __visibles)) _visibles)
  (:geometry_types
   (&optional __geometry_types)
   (if __geometry_types (setq _geometry_types __geometry_types)) _geometry_types)
  (:mesh_geom_filenames
   (&optional __mesh_geom_filenames)
   (if __mesh_geom_filenames (setq _mesh_geom_filenames __mesh_geom_filenames)) _mesh_geom_filenames)
  (:mesh_geom_scales
   (&rest __mesh_geom_scales)
   (if (keywordp (car __mesh_geom_scales))
       (send* _mesh_geom_scales __mesh_geom_scales)
     (progn
       (if __mesh_geom_scales (setq _mesh_geom_scales (car __mesh_geom_scales)))
       _mesh_geom_scales)))
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
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
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; string[] _visual_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _visual_names)) 4
    ;; std_msgs/ColorRGBA[] _ambients
    (apply #'+ (send-all _ambients :serialization-length)) 4
    ;; std_msgs/ColorRGBA[] _diffuses
    (apply #'+ (send-all _diffuses :serialization-length)) 4
    ;; std_msgs/ColorRGBA[] _speculars
    (apply #'+ (send-all _speculars :serialization-length)) 4
    ;; std_msgs/ColorRGBA[] _emissives
    (apply #'+ (send-all _emissives :serialization-length)) 4
    ;; float64[] _transparencies
    (* 8    (length _transparencies)) 4
    ;; int8[] _visibles
    (* 1    (length _visibles)) 4
    ;; uint16[] _geometry_types
    (* 2    (length _geometry_types)) 4
    ;; string[] _mesh_geom_filenames
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _mesh_geom_filenames)) 4
    ;; geometry_msgs/Vector3[] _mesh_geom_scales
    (apply #'+ (send-all _mesh_geom_scales :serialization-length)) 4
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
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
     ;; std_msgs/ColorRGBA[] _ambients
     (write-long (length _ambients) s)
     (dolist (elem _ambients)
       (send elem :serialize s)
       )
     ;; std_msgs/ColorRGBA[] _diffuses
     (write-long (length _diffuses) s)
     (dolist (elem _diffuses)
       (send elem :serialize s)
       )
     ;; std_msgs/ColorRGBA[] _speculars
     (write-long (length _speculars) s)
     (dolist (elem _speculars)
       (send elem :serialize s)
       )
     ;; std_msgs/ColorRGBA[] _emissives
     (write-long (length _emissives) s)
     (dolist (elem _emissives)
       (send elem :serialize s)
       )
     ;; float64[] _transparencies
     (write-long (length _transparencies) s)
     (dotimes (i (length _transparencies))
       (sys::poke (elt _transparencies i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8[] _visibles
     (write-long (length _visibles) s)
     (dotimes (i (length _visibles))
       (write-byte (elt _visibles i) s)
       )
     ;; uint16[] _geometry_types
     (write-long (length _geometry_types) s)
     (dotimes (i (length _geometry_types))
       (write-word (elt _geometry_types i) s)
       )
     ;; string[] _mesh_geom_filenames
     (write-long (length _mesh_geom_filenames) s)
     (dolist (elem _mesh_geom_filenames)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Vector3[] _mesh_geom_scales
     (write-long (length _mesh_geom_scales) s)
     (dolist (elem _mesh_geom_scales)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
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
   ;; std_msgs/ColorRGBA[] _ambients
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ambients (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _ambients)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/ColorRGBA[] _diffuses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _diffuses (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _diffuses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/ColorRGBA[] _speculars
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speculars (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _speculars)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/ColorRGBA[] _emissives
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _emissives (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _emissives)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _transparencies
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _transparencies (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _transparencies i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int8[] _visibles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _visibles (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _visibles i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _visibles i) 127) (setf (elt _visibles i) (- (elt _visibles i) 256)))
     ))
   ;; uint16[] _geometry_types
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _geometry_types (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _geometry_types i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; string[] _mesh_geom_filenames
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mesh_geom_filenames (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _mesh_geom_filenames i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Vector3[] _mesh_geom_scales
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mesh_geom_scales (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _mesh_geom_scales)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
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

(defclass deepracer_msgs::GetVisuals
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::GetVisuals :md5sum-) "aae1fc3904e0c0f096374889f5dbd21f")
(setf (get deepracer_msgs::GetVisuals :datatype-) "deepracer_msgs/GetVisuals")
(setf (get deepracer_msgs::GetVisuals :request) deepracer_msgs::GetVisualsRequest)
(setf (get deepracer_msgs::GetVisuals :response) deepracer_msgs::GetVisualsResponse)

(defmethod deepracer_msgs::GetVisualsRequest
  (:response () (instance deepracer_msgs::GetVisualsResponse :init)))

(setf (get deepracer_msgs::GetVisualsRequest :md5sum-) "aae1fc3904e0c0f096374889f5dbd21f")
(setf (get deepracer_msgs::GetVisualsRequest :datatype-) "deepracer_msgs/GetVisualsRequest")
(setf (get deepracer_msgs::GetVisualsRequest :definition-)
      "string[] link_names
string[] visual_names
---
string[] link_names
string[] visual_names
std_msgs/ColorRGBA[] ambients
std_msgs/ColorRGBA[] diffuses
std_msgs/ColorRGBA[] speculars
std_msgs/ColorRGBA[] emissives
float64[] transparencies
int8[] visibles
uint16[] geometry_types
string[] mesh_geom_filenames
geometry_msgs/Vector3[] mesh_geom_scales
geometry_msgs/Pose[] poses
bool success
string status_message
int8[] status
string[] messages

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

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
float64 z
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
")

(setf (get deepracer_msgs::GetVisualsResponse :md5sum-) "aae1fc3904e0c0f096374889f5dbd21f")
(setf (get deepracer_msgs::GetVisualsResponse :datatype-) "deepracer_msgs/GetVisualsResponse")
(setf (get deepracer_msgs::GetVisualsResponse :definition-)
      "string[] link_names
string[] visual_names
---
string[] link_names
string[] visual_names
std_msgs/ColorRGBA[] ambients
std_msgs/ColorRGBA[] diffuses
std_msgs/ColorRGBA[] speculars
std_msgs/ColorRGBA[] emissives
float64[] transparencies
int8[] visibles
uint16[] geometry_types
string[] mesh_geom_filenames
geometry_msgs/Vector3[] mesh_geom_scales
geometry_msgs/Pose[] poses
bool success
string status_message
int8[] status
string[] messages

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

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
float64 z
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
")



(provide :deepracer_msgs/GetVisuals "aae1fc3904e0c0f096374889f5dbd21f")


