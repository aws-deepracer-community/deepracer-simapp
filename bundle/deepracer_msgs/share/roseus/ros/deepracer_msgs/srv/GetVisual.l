;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::GetVisual)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'GetVisual (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::GETVISUAL")
  (make-package "DEEPRACER_MSGS::GETVISUAL"))
(unless (find-package "DEEPRACER_MSGS::GETVISUALREQUEST")
  (make-package "DEEPRACER_MSGS::GETVISUALREQUEST"))
(unless (find-package "DEEPRACER_MSGS::GETVISUALRESPONSE")
  (make-package "DEEPRACER_MSGS::GETVISUALRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass deepracer_msgs::GetVisualRequest
  :super ros::object
  :slots (_link_name _visual_name ))

(defmethod deepracer_msgs::GetVisualRequest
  (:init
   (&key
    ((:link_name __link_name) "")
    ((:visual_name __visual_name) "")
    )
   (send-super :init)
   (setq _link_name (string __link_name))
   (setq _visual_name (string __visual_name))
   self)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:visual_name
   (&optional __visual_name)
   (if __visual_name (setq _visual_name __visual_name)) _visual_name)
  (:serialization-length
   ()
   (+
    ;; string _link_name
    4 (length _link_name)
    ;; string _visual_name
    4 (length _visual_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _link_name
       (write-long (length _link_name) s) (princ _link_name s)
     ;; string _visual_name
       (write-long (length _visual_name) s) (princ _visual_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _link_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _visual_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _visual_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_msgs::GetVisualResponse
  :super ros::object
  :slots (_ambient _diffuse _specular _emissive _transparency _visible _geometry_type _mesh_geom_filename _mesh_geom_scale _pose _success _status_message ))

(defmethod deepracer_msgs::GetVisualResponse
  (:init
   (&key
    ((:ambient __ambient) (instance std_msgs::ColorRGBA :init))
    ((:diffuse __diffuse) (instance std_msgs::ColorRGBA :init))
    ((:specular __specular) (instance std_msgs::ColorRGBA :init))
    ((:emissive __emissive) (instance std_msgs::ColorRGBA :init))
    ((:transparency __transparency) 0.0)
    ((:visible __visible) nil)
    ((:geometry_type __geometry_type) 0)
    ((:mesh_geom_filename __mesh_geom_filename) "")
    ((:mesh_geom_scale __mesh_geom_scale) (instance geometry_msgs::Vector3 :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _ambient __ambient)
   (setq _diffuse __diffuse)
   (setq _specular __specular)
   (setq _emissive __emissive)
   (setq _transparency (float __transparency))
   (setq _visible __visible)
   (setq _geometry_type (round __geometry_type))
   (setq _mesh_geom_filename (string __mesh_geom_filename))
   (setq _mesh_geom_scale __mesh_geom_scale)
   (setq _pose __pose)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:ambient
   (&rest __ambient)
   (if (keywordp (car __ambient))
       (send* _ambient __ambient)
     (progn
       (if __ambient (setq _ambient (car __ambient)))
       _ambient)))
  (:diffuse
   (&rest __diffuse)
   (if (keywordp (car __diffuse))
       (send* _diffuse __diffuse)
     (progn
       (if __diffuse (setq _diffuse (car __diffuse)))
       _diffuse)))
  (:specular
   (&rest __specular)
   (if (keywordp (car __specular))
       (send* _specular __specular)
     (progn
       (if __specular (setq _specular (car __specular)))
       _specular)))
  (:emissive
   (&rest __emissive)
   (if (keywordp (car __emissive))
       (send* _emissive __emissive)
     (progn
       (if __emissive (setq _emissive (car __emissive)))
       _emissive)))
  (:transparency
   (&optional __transparency)
   (if __transparency (setq _transparency __transparency)) _transparency)
  (:visible
   (&optional __visible)
   (if __visible (setq _visible __visible)) _visible)
  (:geometry_type
   (&optional __geometry_type)
   (if __geometry_type (setq _geometry_type __geometry_type)) _geometry_type)
  (:mesh_geom_filename
   (&optional __mesh_geom_filename)
   (if __mesh_geom_filename (setq _mesh_geom_filename __mesh_geom_filename)) _mesh_geom_filename)
  (:mesh_geom_scale
   (&rest __mesh_geom_scale)
   (if (keywordp (car __mesh_geom_scale))
       (send* _mesh_geom_scale __mesh_geom_scale)
     (progn
       (if __mesh_geom_scale (setq _mesh_geom_scale (car __mesh_geom_scale)))
       _mesh_geom_scale)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; std_msgs/ColorRGBA _ambient
    (send _ambient :serialization-length)
    ;; std_msgs/ColorRGBA _diffuse
    (send _diffuse :serialization-length)
    ;; std_msgs/ColorRGBA _specular
    (send _specular :serialization-length)
    ;; std_msgs/ColorRGBA _emissive
    (send _emissive :serialization-length)
    ;; float64 _transparency
    8
    ;; bool _visible
    1
    ;; uint16 _geometry_type
    2
    ;; string _mesh_geom_filename
    4 (length _mesh_geom_filename)
    ;; geometry_msgs/Vector3 _mesh_geom_scale
    (send _mesh_geom_scale :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/ColorRGBA _ambient
       (send _ambient :serialize s)
     ;; std_msgs/ColorRGBA _diffuse
       (send _diffuse :serialize s)
     ;; std_msgs/ColorRGBA _specular
       (send _specular :serialize s)
     ;; std_msgs/ColorRGBA _emissive
       (send _emissive :serialize s)
     ;; float64 _transparency
       (sys::poke _transparency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _visible
       (if _visible (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _geometry_type
       (write-word _geometry_type s)
     ;; string _mesh_geom_filename
       (write-long (length _mesh_geom_filename) s) (princ _mesh_geom_filename s)
     ;; geometry_msgs/Vector3 _mesh_geom_scale
       (send _mesh_geom_scale :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/ColorRGBA _ambient
     (send _ambient :deserialize buf ptr-) (incf ptr- (send _ambient :serialization-length))
   ;; std_msgs/ColorRGBA _diffuse
     (send _diffuse :deserialize buf ptr-) (incf ptr- (send _diffuse :serialization-length))
   ;; std_msgs/ColorRGBA _specular
     (send _specular :deserialize buf ptr-) (incf ptr- (send _specular :serialization-length))
   ;; std_msgs/ColorRGBA _emissive
     (send _emissive :deserialize buf ptr-) (incf ptr- (send _emissive :serialization-length))
   ;; float64 _transparency
     (setq _transparency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _visible
     (setq _visible (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _geometry_type
     (setq _geometry_type (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _mesh_geom_filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mesh_geom_filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _mesh_geom_scale
     (send _mesh_geom_scale :deserialize buf ptr-) (incf ptr- (send _mesh_geom_scale :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_msgs::GetVisual
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::GetVisual :md5sum-) "addaab363bcf820667e503bbd31b4f3d")
(setf (get deepracer_msgs::GetVisual :datatype-) "deepracer_msgs/GetVisual")
(setf (get deepracer_msgs::GetVisual :request) deepracer_msgs::GetVisualRequest)
(setf (get deepracer_msgs::GetVisual :response) deepracer_msgs::GetVisualResponse)

(defmethod deepracer_msgs::GetVisualRequest
  (:response () (instance deepracer_msgs::GetVisualResponse :init)))

(setf (get deepracer_msgs::GetVisualRequest :md5sum-) "addaab363bcf820667e503bbd31b4f3d")
(setf (get deepracer_msgs::GetVisualRequest :datatype-) "deepracer_msgs/GetVisualRequest")
(setf (get deepracer_msgs::GetVisualRequest :definition-)
      "string link_name
string visual_name
---
std_msgs/ColorRGBA ambient
std_msgs/ColorRGBA diffuse
std_msgs/ColorRGBA specular
std_msgs/ColorRGBA emissive
float64 transparency
bool visible
uint16 geometry_type
string mesh_geom_filename
geometry_msgs/Vector3 mesh_geom_scale
geometry_msgs/Pose pose
bool success
string status_message

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

(setf (get deepracer_msgs::GetVisualResponse :md5sum-) "addaab363bcf820667e503bbd31b4f3d")
(setf (get deepracer_msgs::GetVisualResponse :datatype-) "deepracer_msgs/GetVisualResponse")
(setf (get deepracer_msgs::GetVisualResponse :definition-)
      "string link_name
string visual_name
---
std_msgs/ColorRGBA ambient
std_msgs/ColorRGBA diffuse
std_msgs/ColorRGBA specular
std_msgs/ColorRGBA emissive
float64 transparency
bool visible
uint16 geometry_type
string mesh_geom_filename
geometry_msgs/Vector3 mesh_geom_scale
geometry_msgs/Pose pose
bool success
string status_message

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



(provide :deepracer_msgs/GetVisual "addaab363bcf820667e503bbd31b4f3d")


