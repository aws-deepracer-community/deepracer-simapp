; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude SetVisualColor-request.msg.html

(cl:defclass <SetVisualColor-request> (roslisp-msg-protocol:ros-message)
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
   (ambient
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
   (block
    :reader block
    :initarg :block
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVisualColor-request (<SetVisualColor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualColor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualColor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualColor-request> is deprecated: use deepracer_msgs-srv:SetVisualColor-request instead.")))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_name-val is deprecated.  Use deepracer_msgs-srv:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'visual_name-val :lambda-list '(m))
(cl:defmethod visual_name-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visual_name-val is deprecated.  Use deepracer_msgs-srv:visual_name instead.")
  (visual_name m))

(cl:ensure-generic-function 'ambient-val :lambda-list '(m))
(cl:defmethod ambient-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:ambient-val is deprecated.  Use deepracer_msgs-srv:ambient instead.")
  (ambient m))

(cl:ensure-generic-function 'diffuse-val :lambda-list '(m))
(cl:defmethod diffuse-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:diffuse-val is deprecated.  Use deepracer_msgs-srv:diffuse instead.")
  (diffuse m))

(cl:ensure-generic-function 'specular-val :lambda-list '(m))
(cl:defmethod specular-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:specular-val is deprecated.  Use deepracer_msgs-srv:specular instead.")
  (specular m))

(cl:ensure-generic-function 'emissive-val :lambda-list '(m))
(cl:defmethod emissive-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:emissive-val is deprecated.  Use deepracer_msgs-srv:emissive instead.")
  (emissive m))

(cl:ensure-generic-function 'block-val :lambda-list '(m))
(cl:defmethod block-val ((m <SetVisualColor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:block-val is deprecated.  Use deepracer_msgs-srv:block instead.")
  (block m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualColor-request>) ostream)
  "Serializes a message object of type '<SetVisualColor-request>"
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ambient) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'diffuse) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'specular) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emissive) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'block) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualColor-request>) istream)
  "Deserializes a message object of type '<SetVisualColor-request>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ambient) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'diffuse) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'specular) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emissive) istream)
    (cl:setf (cl:slot-value msg 'block) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualColor-request>)))
  "Returns string type for a service object of type '<SetVisualColor-request>"
  "deepracer_msgs/SetVisualColorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualColor-request)))
  "Returns string type for a service object of type 'SetVisualColor-request"
  "deepracer_msgs/SetVisualColorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualColor-request>)))
  "Returns md5sum for a message object of type '<SetVisualColor-request>"
  "9c987659e93e8e993b90a6ea6fab5b74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualColor-request)))
  "Returns md5sum for a message object of type 'SetVisualColor-request"
  "9c987659e93e8e993b90a6ea6fab5b74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualColor-request>)))
  "Returns full string definition for message of type '<SetVisualColor-request>"
  (cl:format cl:nil "string link_name~%string visual_name~%std_msgs/ColorRGBA ambient~%std_msgs/ColorRGBA diffuse~%std_msgs/ColorRGBA specular~%std_msgs/ColorRGBA emissive~%bool block~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualColor-request)))
  "Returns full string definition for message of type 'SetVisualColor-request"
  (cl:format cl:nil "string link_name~%string visual_name~%std_msgs/ColorRGBA ambient~%std_msgs/ColorRGBA diffuse~%std_msgs/ColorRGBA specular~%std_msgs/ColorRGBA emissive~%bool block~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualColor-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'link_name))
     4 (cl:length (cl:slot-value msg 'visual_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ambient))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'diffuse))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'specular))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emissive))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualColor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualColor-request
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':visual_name (visual_name msg))
    (cl:cons ':ambient (ambient msg))
    (cl:cons ':diffuse (diffuse msg))
    (cl:cons ':specular (specular msg))
    (cl:cons ':emissive (emissive msg))
    (cl:cons ':block (block msg))
))
;//! \htmlinclude SetVisualColor-response.msg.html

(cl:defclass <SetVisualColor-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetVisualColor-response (<SetVisualColor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualColor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualColor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualColor-response> is deprecated: use deepracer_msgs-srv:SetVisualColor-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetVisualColor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetVisualColor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualColor-response>) ostream)
  "Serializes a message object of type '<SetVisualColor-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualColor-response>) istream)
  "Deserializes a message object of type '<SetVisualColor-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualColor-response>)))
  "Returns string type for a service object of type '<SetVisualColor-response>"
  "deepracer_msgs/SetVisualColorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualColor-response)))
  "Returns string type for a service object of type 'SetVisualColor-response"
  "deepracer_msgs/SetVisualColorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualColor-response>)))
  "Returns md5sum for a message object of type '<SetVisualColor-response>"
  "9c987659e93e8e993b90a6ea6fab5b74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualColor-response)))
  "Returns md5sum for a message object of type 'SetVisualColor-response"
  "9c987659e93e8e993b90a6ea6fab5b74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualColor-response>)))
  "Returns full string definition for message of type '<SetVisualColor-response>"
  (cl:format cl:nil "bool success~%string status_message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualColor-response)))
  "Returns full string definition for message of type 'SetVisualColor-response"
  (cl:format cl:nil "bool success~%string status_message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualColor-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualColor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualColor-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVisualColor)))
  'SetVisualColor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVisualColor)))
  'SetVisualColor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualColor)))
  "Returns string type for a service object of type '<SetVisualColor>"
  "deepracer_msgs/SetVisualColor")