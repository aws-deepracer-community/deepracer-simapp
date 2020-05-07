; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude SetVisualMesh-request.msg.html

(cl:defclass <SetVisualMesh-request> (roslisp-msg-protocol:ros-message)
  ((link_name
    :reader link_name
    :initarg :link_name
    :type cl:string
    :initform "")
   (visual_name
    :reader visual_name
    :initarg :visual_name
    :type cl:string
    :initform "")
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (scale
    :reader scale
    :initarg :scale
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (block
    :reader block
    :initarg :block
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVisualMesh-request (<SetVisualMesh-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualMesh-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualMesh-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualMesh-request> is deprecated: use deepracer_msgs-srv:SetVisualMesh-request instead.")))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <SetVisualMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_name-val is deprecated.  Use deepracer_msgs-srv:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'visual_name-val :lambda-list '(m))
(cl:defmethod visual_name-val ((m <SetVisualMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visual_name-val is deprecated.  Use deepracer_msgs-srv:visual_name instead.")
  (visual_name m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <SetVisualMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:filename-val is deprecated.  Use deepracer_msgs-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <SetVisualMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:scale-val is deprecated.  Use deepracer_msgs-srv:scale instead.")
  (scale m))

(cl:ensure-generic-function 'block-val :lambda-list '(m))
(cl:defmethod block-val ((m <SetVisualMesh-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:block-val is deprecated.  Use deepracer_msgs-srv:block instead.")
  (block m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualMesh-request>) ostream)
  "Serializes a message object of type '<SetVisualMesh-request>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'scale) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'block) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualMesh-request>) istream)
  "Deserializes a message object of type '<SetVisualMesh-request>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'scale) istream)
    (cl:setf (cl:slot-value msg 'block) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualMesh-request>)))
  "Returns string type for a service object of type '<SetVisualMesh-request>"
  "deepracer_msgs/SetVisualMeshRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualMesh-request)))
  "Returns string type for a service object of type 'SetVisualMesh-request"
  "deepracer_msgs/SetVisualMeshRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualMesh-request>)))
  "Returns md5sum for a message object of type '<SetVisualMesh-request>"
  "f2c3b05d1c4b997abfc5202f04e76d70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualMesh-request)))
  "Returns md5sum for a message object of type 'SetVisualMesh-request"
  "f2c3b05d1c4b997abfc5202f04e76d70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualMesh-request>)))
  "Returns full string definition for message of type '<SetVisualMesh-request>"
  (cl:format cl:nil "string link_name~%string visual_name~%string filename~%geometry_msgs/Vector3 scale~%bool block~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualMesh-request)))
  "Returns full string definition for message of type 'SetVisualMesh-request"
  (cl:format cl:nil "string link_name~%string visual_name~%string filename~%geometry_msgs/Vector3 scale~%bool block~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualMesh-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'link_name))
     4 (cl:length (cl:slot-value msg 'visual_name))
     4 (cl:length (cl:slot-value msg 'filename))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'scale))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualMesh-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualMesh-request
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':visual_name (visual_name msg))
    (cl:cons ':filename (filename msg))
    (cl:cons ':scale (scale msg))
    (cl:cons ':block (block msg))
))
;//! \htmlinclude SetVisualMesh-response.msg.html

(cl:defclass <SetVisualMesh-response> (roslisp-msg-protocol:ros-message)
  ((success
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

(cl:defclass SetVisualMesh-response (<SetVisualMesh-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualMesh-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualMesh-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualMesh-response> is deprecated: use deepracer_msgs-srv:SetVisualMesh-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetVisualMesh-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetVisualMesh-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualMesh-response>) ostream)
  "Serializes a message object of type '<SetVisualMesh-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualMesh-response>) istream)
  "Deserializes a message object of type '<SetVisualMesh-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualMesh-response>)))
  "Returns string type for a service object of type '<SetVisualMesh-response>"
  "deepracer_msgs/SetVisualMeshResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualMesh-response)))
  "Returns string type for a service object of type 'SetVisualMesh-response"
  "deepracer_msgs/SetVisualMeshResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualMesh-response>)))
  "Returns md5sum for a message object of type '<SetVisualMesh-response>"
  "f2c3b05d1c4b997abfc5202f04e76d70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualMesh-response)))
  "Returns md5sum for a message object of type 'SetVisualMesh-response"
  "f2c3b05d1c4b997abfc5202f04e76d70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualMesh-response>)))
  "Returns full string definition for message of type '<SetVisualMesh-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualMesh-response)))
  "Returns full string definition for message of type 'SetVisualMesh-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualMesh-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualMesh-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualMesh-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVisualMesh)))
  'SetVisualMesh-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVisualMesh)))
  'SetVisualMesh-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualMesh)))
  "Returns string type for a service object of type '<SetVisualMesh>"
  "deepracer_msgs/SetVisualMesh")