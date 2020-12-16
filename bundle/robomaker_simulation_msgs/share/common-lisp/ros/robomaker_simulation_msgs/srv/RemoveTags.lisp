; Auto-generated. Do not edit!


(cl:in-package robomaker_simulation_msgs-srv)


;//! \htmlinclude RemoveTags-request.msg.html

(cl:defclass <RemoveTags-request> (roslisp-msg-protocol:ros-message)
  ((keys
    :reader keys
    :initarg :keys
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass RemoveTags-request (<RemoveTags-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveTags-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveTags-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-srv:<RemoveTags-request> is deprecated: use robomaker_simulation_msgs-srv:RemoveTags-request instead.")))

(cl:ensure-generic-function 'keys-val :lambda-list '(m))
(cl:defmethod keys-val ((m <RemoveTags-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:keys-val is deprecated.  Use robomaker_simulation_msgs-srv:keys instead.")
  (keys m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveTags-request>) ostream)
  "Serializes a message object of type '<RemoveTags-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'keys))))
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
   (cl:slot-value msg 'keys))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveTags-request>) istream)
  "Deserializes a message object of type '<RemoveTags-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'keys) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'keys)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveTags-request>)))
  "Returns string type for a service object of type '<RemoveTags-request>"
  "robomaker_simulation_msgs/RemoveTagsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveTags-request)))
  "Returns string type for a service object of type 'RemoveTags-request"
  "robomaker_simulation_msgs/RemoveTagsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveTags-request>)))
  "Returns md5sum for a message object of type '<RemoveTags-request>"
  "cae332c01603844d352ed026669010ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveTags-request)))
  "Returns md5sum for a message object of type 'RemoveTags-request"
  "cae332c01603844d352ed026669010ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveTags-request>)))
  "Returns full string definition for message of type '<RemoveTags-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string[] keys~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveTags-request)))
  "Returns full string definition for message of type 'RemoveTags-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string[] keys~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveTags-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'keys) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveTags-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveTags-request
    (cl:cons ':keys (keys msg))
))
;//! \htmlinclude RemoveTags-response.msg.html

(cl:defclass <RemoveTags-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass RemoveTags-response (<RemoveTags-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RemoveTags-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RemoveTags-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-srv:<RemoveTags-response> is deprecated: use robomaker_simulation_msgs-srv:RemoveTags-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RemoveTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:success-val is deprecated.  Use robomaker_simulation_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <RemoveTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:message-val is deprecated.  Use robomaker_simulation_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RemoveTags-response>) ostream)
  "Serializes a message object of type '<RemoveTags-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RemoveTags-response>) istream)
  "Deserializes a message object of type '<RemoveTags-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RemoveTags-response>)))
  "Returns string type for a service object of type '<RemoveTags-response>"
  "robomaker_simulation_msgs/RemoveTagsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveTags-response)))
  "Returns string type for a service object of type 'RemoveTags-response"
  "robomaker_simulation_msgs/RemoveTagsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RemoveTags-response>)))
  "Returns md5sum for a message object of type '<RemoveTags-response>"
  "cae332c01603844d352ed026669010ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RemoveTags-response)))
  "Returns md5sum for a message object of type 'RemoveTags-response"
  "cae332c01603844d352ed026669010ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RemoveTags-response>)))
  "Returns full string definition for message of type '<RemoveTags-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RemoveTags-response)))
  "Returns full string definition for message of type 'RemoveTags-response"
  (cl:format cl:nil "~%bool success~%~%~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RemoveTags-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RemoveTags-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RemoveTags-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RemoveTags)))
  'RemoveTags-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RemoveTags)))
  'RemoveTags-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RemoveTags)))
  "Returns string type for a service object of type '<RemoveTags>"
  "robomaker_simulation_msgs/RemoveTags")