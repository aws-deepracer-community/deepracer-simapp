; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude GetVisual-request.msg.html

(cl:defclass <GetVisual-request> (roslisp-msg-protocol:ros-message)
  ((link_name
    :reader link_name
    :initarg :link_name
    :type cl:string
    :initform "")
   (visual_name
    :reader visual_name
    :initarg :visual_name
    :type cl:string
    :initform ""))
)

(cl:defclass GetVisual-request (<GetVisual-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVisual-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVisual-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<GetVisual-request> is deprecated: use deepracer_msgs-srv:GetVisual-request instead.")))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <GetVisual-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_name-val is deprecated.  Use deepracer_msgs-srv:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'visual_name-val :lambda-list '(m))
(cl:defmethod visual_name-val ((m <GetVisual-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visual_name-val is deprecated.  Use deepracer_msgs-srv:visual_name instead.")
  (visual_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVisual-request>) ostream)
  "Serializes a message object of type '<GetVisual-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'visual_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'visual_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVisual-request>) istream)
  "Deserializes a message object of type '<GetVisual-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'visual_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'visual_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVisual-request>)))
  "Returns string type for a service object of type '<GetVisual-request>"
  "deepracer_msgs/GetVisualRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVisual-request)))
  "Returns string type for a service object of type 'GetVisual-request"
  "deepracer_msgs/GetVisualRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVisual-request>)))
  "Returns md5sum for a message object of type '<GetVisual-request>"
  "addaab363bcf820667e503bbd31b4f3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVisual-request)))
  "Returns md5sum for a message object of type 'GetVisual-request"
  "addaab363bcf820667e503bbd31b4f3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVisual-request>)))
  "Returns full string definition for message of type '<GetVisual-request>"
  (cl:format cl:nil "string link_name~%string visual_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVisual-request)))
  "Returns full string definition for message of type 'GetVisual-request"
  (cl:format cl:nil "string link_name~%string visual_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVisual-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'link_name))
     4 (cl:length (cl:slot-value msg 'visual_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVisual-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVisual-request
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':visual_name (visual_name msg))
))
;//! \htmlinclude GetVisual-response.msg.html

(cl:defclass <GetVisual-response> (roslisp-msg-protocol:ros-message)
  ((ambient
    :reader ambient
    :initarg :ambient
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (diffuse
    :reader diffuse
    :initarg :diffuse
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (specular
    :reader specular
    :initarg :specular
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (emissive
    :reader emissive
    :initarg :emissive
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (transparency
    :reader transparency
    :initarg :transparency
    :type cl:float
    :initform 0.0)
   (visible
    :reader visible
    :initarg :visible
    :type cl:boolean
    :initform cl:nil)
   (geometry_type
    :reader geometry_type
    :initarg :geometry_type
    :type cl:fixnum
    :initform 0)
   (mesh_geom_filename
    :reader mesh_geom_filename
    :initarg :mesh_geom_filename
    :type cl:string
    :initform "")
   (mesh_geom_scale
    :reader mesh_geom_scale
    :initarg :mesh_geom_scale
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status_message
    :reader status_message
    :initarg :status_message
    :type cl:string
    :initform ""))
)

(cl:defclass GetVisual-response (<GetVisual-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVisual-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVisual-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<GetVisual-response> is deprecated: use deepracer_msgs-srv:GetVisual-response instead.")))

(cl:ensure-generic-function 'ambient-val :lambda-list '(m))
(cl:defmethod ambient-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:ambient-val is deprecated.  Use deepracer_msgs-srv:ambient instead.")
  (ambient m))

(cl:ensure-generic-function 'diffuse-val :lambda-list '(m))
(cl:defmethod diffuse-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:diffuse-val is deprecated.  Use deepracer_msgs-srv:diffuse instead.")
  (diffuse m))

(cl:ensure-generic-function 'specular-val :lambda-list '(m))
(cl:defmethod specular-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:specular-val is deprecated.  Use deepracer_msgs-srv:specular instead.")
  (specular m))

(cl:ensure-generic-function 'emissive-val :lambda-list '(m))
(cl:defmethod emissive-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:emissive-val is deprecated.  Use deepracer_msgs-srv:emissive instead.")
  (emissive m))

(cl:ensure-generic-function 'transparency-val :lambda-list '(m))
(cl:defmethod transparency-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:transparency-val is deprecated.  Use deepracer_msgs-srv:transparency instead.")
  (transparency m))

(cl:ensure-generic-function 'visible-val :lambda-list '(m))
(cl:defmethod visible-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visible-val is deprecated.  Use deepracer_msgs-srv:visible instead.")
  (visible m))

(cl:ensure-generic-function 'geometry_type-val :lambda-list '(m))
(cl:defmethod geometry_type-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:geometry_type-val is deprecated.  Use deepracer_msgs-srv:geometry_type instead.")
  (geometry_type m))

(cl:ensure-generic-function 'mesh_geom_filename-val :lambda-list '(m))
(cl:defmethod mesh_geom_filename-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:mesh_geom_filename-val is deprecated.  Use deepracer_msgs-srv:mesh_geom_filename instead.")
  (mesh_geom_filename m))

(cl:ensure-generic-function 'mesh_geom_scale-val :lambda-list '(m))
(cl:defmethod mesh_geom_scale-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:mesh_geom_scale-val is deprecated.  Use deepracer_msgs-srv:mesh_geom_scale instead.")
  (mesh_geom_scale m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:pose-val is deprecated.  Use deepracer_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <GetVisual-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVisual-response>) ostream)
  "Serializes a message object of type '<GetVisual-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ambient) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'diffuse) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'specular) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emissive) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'transparency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'visible) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'geometry_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'geometry_type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mesh_geom_filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mesh_geom_filename))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mesh_geom_scale) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVisual-response>) istream)
  "Deserializes a message object of type '<GetVisual-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ambient) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'diffuse) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'specular) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emissive) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'transparency) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'visible) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'geometry_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'geometry_type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mesh_geom_filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mesh_geom_filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mesh_geom_scale) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVisual-response>)))
  "Returns string type for a service object of type '<GetVisual-response>"
  "deepracer_msgs/GetVisualResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVisual-response)))
  "Returns string type for a service object of type 'GetVisual-response"
  "deepracer_msgs/GetVisualResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVisual-response>)))
  "Returns md5sum for a message object of type '<GetVisual-response>"
  "addaab363bcf820667e503bbd31b4f3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVisual-response)))
  "Returns md5sum for a message object of type 'GetVisual-response"
  "addaab363bcf820667e503bbd31b4f3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVisual-response>)))
  "Returns full string definition for message of type '<GetVisual-response>"
  (cl:format cl:nil "std_msgs/ColorRGBA ambient~%std_msgs/ColorRGBA diffuse~%std_msgs/ColorRGBA specular~%std_msgs/ColorRGBA emissive~%float64 transparency~%bool visible~%uint16 geometry_type~%string mesh_geom_filename~%geometry_msgs/Vector3 mesh_geom_scale~%geometry_msgs/Pose pose~%bool success~%string status_message~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVisual-response)))
  "Returns full string definition for message of type 'GetVisual-response"
  (cl:format cl:nil "std_msgs/ColorRGBA ambient~%std_msgs/ColorRGBA diffuse~%std_msgs/ColorRGBA specular~%std_msgs/ColorRGBA emissive~%float64 transparency~%bool visible~%uint16 geometry_type~%string mesh_geom_filename~%geometry_msgs/Vector3 mesh_geom_scale~%geometry_msgs/Pose pose~%bool success~%string status_message~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVisual-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ambient))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'diffuse))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'specular))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emissive))
     8
     1
     2
     4 (cl:length (cl:slot-value msg 'mesh_geom_filename))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mesh_geom_scale))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVisual-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVisual-response
    (cl:cons ':ambient (ambient msg))
    (cl:cons ':diffuse (diffuse msg))
    (cl:cons ':specular (specular msg))
    (cl:cons ':emissive (emissive msg))
    (cl:cons ':transparency (transparency msg))
    (cl:cons ':visible (visible msg))
    (cl:cons ':geometry_type (geometry_type msg))
    (cl:cons ':mesh_geom_filename (mesh_geom_filename msg))
    (cl:cons ':mesh_geom_scale (mesh_geom_scale msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetVisual)))
  'GetVisual-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetVisual)))
  'GetVisual-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVisual)))
  "Returns string type for a service object of type '<GetVisual>"
  "deepracer_msgs/GetVisual")