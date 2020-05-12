; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude GetLightNames-request.msg.html

(cl:defclass <GetLightNames-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetLightNames-request (<GetLightNames-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLightNames-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLightNames-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<GetLightNames-request> is deprecated: use deepracer_msgs-srv:GetLightNames-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLightNames-request>) ostream)
  "Serializes a message object of type '<GetLightNames-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLightNames-request>) istream)
  "Deserializes a message object of type '<GetLightNames-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLightNames-request>)))
  "Returns string type for a service object of type '<GetLightNames-request>"
  "deepracer_msgs/GetLightNamesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLightNames-request)))
  "Returns string type for a service object of type 'GetLightNames-request"
  "deepracer_msgs/GetLightNamesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLightNames-request>)))
  "Returns md5sum for a message object of type '<GetLightNames-request>"
  "6dff1e71fe0fb6f3d275724e4c746d5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLightNames-request)))
  "Returns md5sum for a message object of type 'GetLightNames-request"
  "6dff1e71fe0fb6f3d275724e4c746d5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLightNames-request>)))
  "Returns full string definition for message of type '<GetLightNames-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLightNames-request)))
  "Returns full string definition for message of type 'GetLightNames-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLightNames-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLightNames-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLightNames-request
))
;//! \htmlinclude GetLightNames-response.msg.html

(cl:defclass <GetLightNames-response> (roslisp-msg-protocol:ros-message)
  ((light_names
    :reader light_names
    :initarg :light_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
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

(cl:defclass GetLightNames-response (<GetLightNames-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLightNames-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLightNames-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<GetLightNames-response> is deprecated: use deepracer_msgs-srv:GetLightNames-response instead.")))

(cl:ensure-generic-function 'light_names-val :lambda-list '(m))
(cl:defmethod light_names-val ((m <GetLightNames-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:light_names-val is deprecated.  Use deepracer_msgs-srv:light_names instead.")
  (light_names m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetLightNames-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <GetLightNames-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLightNames-response>) ostream)
  "Serializes a message object of type '<GetLightNames-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'light_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'light_names))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLightNames-response>) istream)
  "Deserializes a message object of type '<GetLightNames-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'light_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'light_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLightNames-response>)))
  "Returns string type for a service object of type '<GetLightNames-response>"
  "deepracer_msgs/GetLightNamesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLightNames-response)))
  "Returns string type for a service object of type 'GetLightNames-response"
  "deepracer_msgs/GetLightNamesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLightNames-response>)))
  "Returns md5sum for a message object of type '<GetLightNames-response>"
  "6dff1e71fe0fb6f3d275724e4c746d5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLightNames-response)))
  "Returns md5sum for a message object of type 'GetLightNames-response"
  "6dff1e71fe0fb6f3d275724e4c746d5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLightNames-response>)))
  "Returns full string definition for message of type '<GetLightNames-response>"
  (cl:format cl:nil "string[] light_names~%bool success~%string status_message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLightNames-response)))
  "Returns full string definition for message of type 'GetLightNames-response"
  (cl:format cl:nil "string[] light_names~%bool success~%string status_message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLightNames-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'light_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     1
     4 (cl:length (cl:slot-value msg 'status_message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLightNames-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLightNames-response
    (cl:cons ':light_names (light_names msg))
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetLightNames)))
  'GetLightNames-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetLightNames)))
  'GetLightNames-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLightNames)))
  "Returns string type for a service object of type '<GetLightNames>"
  "deepracer_msgs/GetLightNames")