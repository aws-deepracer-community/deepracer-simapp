; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude SetVisualPose-request.msg.html

(cl:defclass <SetVisualPose-request> (roslisp-msg-protocol:ros-message)
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
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (block
    :reader block
    :initarg :block
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVisualPose-request (<SetVisualPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualPose-request> is deprecated: use deepracer_msgs-srv:SetVisualPose-request instead.")))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <SetVisualPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_name-val is deprecated.  Use deepracer_msgs-srv:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'visual_name-val :lambda-list '(m))
(cl:defmethod visual_name-val ((m <SetVisualPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visual_name-val is deprecated.  Use deepracer_msgs-srv:visual_name instead.")
  (visual_name m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SetVisualPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:pose-val is deprecated.  Use deepracer_msgs-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'block-val :lambda-list '(m))
(cl:defmethod block-val ((m <SetVisualPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:block-val is deprecated.  Use deepracer_msgs-srv:block instead.")
  (block m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualPose-request>) ostream)
  "Serializes a message object of type '<SetVisualPose-request>"
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'block) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualPose-request>) istream)
  "Deserializes a message object of type '<SetVisualPose-request>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:slot-value msg 'block) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualPose-request>)))
  "Returns string type for a service object of type '<SetVisualPose-request>"
  "deepracer_msgs/SetVisualPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualPose-request)))
  "Returns string type for a service object of type 'SetVisualPose-request"
  "deepracer_msgs/SetVisualPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualPose-request>)))
  "Returns md5sum for a message object of type '<SetVisualPose-request>"
  "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualPose-request)))
  "Returns md5sum for a message object of type 'SetVisualPose-request"
  "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualPose-request>)))
  "Returns full string definition for message of type '<SetVisualPose-request>"
  (cl:format cl:nil "string link_name~%string visual_name~%geometry_msgs/Pose pose~%bool block~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualPose-request)))
  "Returns full string definition for message of type 'SetVisualPose-request"
  (cl:format cl:nil "string link_name~%string visual_name~%geometry_msgs/Pose pose~%bool block~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualPose-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'link_name))
     4 (cl:length (cl:slot-value msg 'visual_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualPose-request
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':visual_name (visual_name msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':block (block msg))
))
;//! \htmlinclude SetVisualPose-response.msg.html

(cl:defclass <SetVisualPose-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetVisualPose-response (<SetVisualPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualPose-response> is deprecated: use deepracer_msgs-srv:SetVisualPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetVisualPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetVisualPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualPose-response>) ostream)
  "Serializes a message object of type '<SetVisualPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualPose-response>) istream)
  "Deserializes a message object of type '<SetVisualPose-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualPose-response>)))
  "Returns string type for a service object of type '<SetVisualPose-response>"
  "deepracer_msgs/SetVisualPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualPose-response)))
  "Returns string type for a service object of type 'SetVisualPose-response"
  "deepracer_msgs/SetVisualPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualPose-response>)))
  "Returns md5sum for a message object of type '<SetVisualPose-response>"
  "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualPose-response)))
  "Returns md5sum for a message object of type 'SetVisualPose-response"
  "3b6fe41d1c80e9b62f81f01d6ed4fc64")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualPose-response>)))
  "Returns full string definition for message of type '<SetVisualPose-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualPose-response)))
  "Returns full string definition for message of type 'SetVisualPose-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualPose-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualPose-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVisualPose)))
  'SetVisualPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVisualPose)))
  'SetVisualPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualPose)))
  "Returns string type for a service object of type '<SetVisualPose>"
  "deepracer_msgs/SetVisualPose")