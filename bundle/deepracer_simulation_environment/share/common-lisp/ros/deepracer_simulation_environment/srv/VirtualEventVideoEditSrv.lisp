; Auto-generated. Do not edit!


(cl:in-package deepracer_simulation_environment-srv)


;//! \htmlinclude VirtualEventVideoEditSrv-request.msg.html

(cl:defclass <VirtualEventVideoEditSrv-request> (roslisp-msg-protocol:ros-message)
  ((display_name
    :reader display_name
    :initarg :display_name
    :type cl:string
    :initform "")
   (racecar_color
    :reader racecar_color
    :initarg :racecar_color
    :type cl:string
    :initform ""))
)

(cl:defclass VirtualEventVideoEditSrv-request (<VirtualEventVideoEditSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VirtualEventVideoEditSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VirtualEventVideoEditSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation_environment-srv:<VirtualEventVideoEditSrv-request> is deprecated: use deepracer_simulation_environment-srv:VirtualEventVideoEditSrv-request instead.")))

(cl:ensure-generic-function 'display_name-val :lambda-list '(m))
(cl:defmethod display_name-val ((m <VirtualEventVideoEditSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:display_name-val is deprecated.  Use deepracer_simulation_environment-srv:display_name instead.")
  (display_name m))

(cl:ensure-generic-function 'racecar_color-val :lambda-list '(m))
(cl:defmethod racecar_color-val ((m <VirtualEventVideoEditSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:racecar_color-val is deprecated.  Use deepracer_simulation_environment-srv:racecar_color instead.")
  (racecar_color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VirtualEventVideoEditSrv-request>) ostream)
  "Serializes a message object of type '<VirtualEventVideoEditSrv-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'display_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'display_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'racecar_color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'racecar_color))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VirtualEventVideoEditSrv-request>) istream)
  "Deserializes a message object of type '<VirtualEventVideoEditSrv-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'display_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'display_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'racecar_color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'racecar_color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VirtualEventVideoEditSrv-request>)))
  "Returns string type for a service object of type '<VirtualEventVideoEditSrv-request>"
  "deepracer_simulation_environment/VirtualEventVideoEditSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VirtualEventVideoEditSrv-request)))
  "Returns string type for a service object of type 'VirtualEventVideoEditSrv-request"
  "deepracer_simulation_environment/VirtualEventVideoEditSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VirtualEventVideoEditSrv-request>)))
  "Returns md5sum for a message object of type '<VirtualEventVideoEditSrv-request>"
  "d375256530e7fab3e8486078c126cdb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VirtualEventVideoEditSrv-request)))
  "Returns md5sum for a message object of type 'VirtualEventVideoEditSrv-request"
  "d375256530e7fab3e8486078c126cdb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VirtualEventVideoEditSrv-request>)))
  "Returns full string definition for message of type '<VirtualEventVideoEditSrv-request>"
  (cl:format cl:nil "string display_name~%string racecar_color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VirtualEventVideoEditSrv-request)))
  "Returns full string definition for message of type 'VirtualEventVideoEditSrv-request"
  (cl:format cl:nil "string display_name~%string racecar_color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VirtualEventVideoEditSrv-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'display_name))
     4 (cl:length (cl:slot-value msg 'racecar_color))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VirtualEventVideoEditSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VirtualEventVideoEditSrv-request
    (cl:cons ':display_name (display_name msg))
    (cl:cons ':racecar_color (racecar_color msg))
))
;//! \htmlinclude VirtualEventVideoEditSrv-response.msg.html

(cl:defclass <VirtualEventVideoEditSrv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VirtualEventVideoEditSrv-response (<VirtualEventVideoEditSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VirtualEventVideoEditSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VirtualEventVideoEditSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation_environment-srv:<VirtualEventVideoEditSrv-response> is deprecated: use deepracer_simulation_environment-srv:VirtualEventVideoEditSrv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <VirtualEventVideoEditSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:success-val is deprecated.  Use deepracer_simulation_environment-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VirtualEventVideoEditSrv-response>) ostream)
  "Serializes a message object of type '<VirtualEventVideoEditSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VirtualEventVideoEditSrv-response>) istream)
  "Deserializes a message object of type '<VirtualEventVideoEditSrv-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VirtualEventVideoEditSrv-response>)))
  "Returns string type for a service object of type '<VirtualEventVideoEditSrv-response>"
  "deepracer_simulation_environment/VirtualEventVideoEditSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VirtualEventVideoEditSrv-response)))
  "Returns string type for a service object of type 'VirtualEventVideoEditSrv-response"
  "deepracer_simulation_environment/VirtualEventVideoEditSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VirtualEventVideoEditSrv-response>)))
  "Returns md5sum for a message object of type '<VirtualEventVideoEditSrv-response>"
  "d375256530e7fab3e8486078c126cdb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VirtualEventVideoEditSrv-response)))
  "Returns md5sum for a message object of type 'VirtualEventVideoEditSrv-response"
  "d375256530e7fab3e8486078c126cdb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VirtualEventVideoEditSrv-response>)))
  "Returns full string definition for message of type '<VirtualEventVideoEditSrv-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VirtualEventVideoEditSrv-response)))
  "Returns full string definition for message of type 'VirtualEventVideoEditSrv-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VirtualEventVideoEditSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VirtualEventVideoEditSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VirtualEventVideoEditSrv-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VirtualEventVideoEditSrv)))
  'VirtualEventVideoEditSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VirtualEventVideoEditSrv)))
  'VirtualEventVideoEditSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VirtualEventVideoEditSrv)))
  "Returns string type for a service object of type '<VirtualEventVideoEditSrv>"
  "deepracer_simulation_environment/VirtualEventVideoEditSrv")