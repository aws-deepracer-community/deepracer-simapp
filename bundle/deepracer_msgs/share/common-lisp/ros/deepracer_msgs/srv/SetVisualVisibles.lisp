; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude SetVisualVisibles-request.msg.html

(cl:defclass <SetVisualVisibles-request> (roslisp-msg-protocol:ros-message)
  ((link_names
    :reader link_names
    :initarg :link_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (visual_names
    :reader visual_names
    :initarg :visual_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (visibles
    :reader visibles
    :initarg :visibles
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (block
    :reader block
    :initarg :block
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetVisualVisibles-request (<SetVisualVisibles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualVisibles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualVisibles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualVisibles-request> is deprecated: use deepracer_msgs-srv:SetVisualVisibles-request instead.")))

(cl:ensure-generic-function 'link_names-val :lambda-list '(m))
(cl:defmethod link_names-val ((m <SetVisualVisibles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_names-val is deprecated.  Use deepracer_msgs-srv:link_names instead.")
  (link_names m))

(cl:ensure-generic-function 'visual_names-val :lambda-list '(m))
(cl:defmethod visual_names-val ((m <SetVisualVisibles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visual_names-val is deprecated.  Use deepracer_msgs-srv:visual_names instead.")
  (visual_names m))

(cl:ensure-generic-function 'visibles-val :lambda-list '(m))
(cl:defmethod visibles-val ((m <SetVisualVisibles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:visibles-val is deprecated.  Use deepracer_msgs-srv:visibles instead.")
  (visibles m))

(cl:ensure-generic-function 'block-val :lambda-list '(m))
(cl:defmethod block-val ((m <SetVisualVisibles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:block-val is deprecated.  Use deepracer_msgs-srv:block instead.")
  (block m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualVisibles-request>) ostream)
  "Serializes a message object of type '<SetVisualVisibles-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'link_names))))
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
   (cl:slot-value msg 'link_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'visual_names))))
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
   (cl:slot-value msg 'visual_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'visibles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'visibles))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'block) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualVisibles-request>) istream)
  "Deserializes a message object of type '<SetVisualVisibles-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'link_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'link_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'visual_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'visual_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'visibles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'visibles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
    (cl:setf (cl:slot-value msg 'block) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualVisibles-request>)))
  "Returns string type for a service object of type '<SetVisualVisibles-request>"
  "deepracer_msgs/SetVisualVisiblesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualVisibles-request)))
  "Returns string type for a service object of type 'SetVisualVisibles-request"
  "deepracer_msgs/SetVisualVisiblesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualVisibles-request>)))
  "Returns md5sum for a message object of type '<SetVisualVisibles-request>"
  "f1147e9c73d95b8955fad12bcf7a1ecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualVisibles-request)))
  "Returns md5sum for a message object of type 'SetVisualVisibles-request"
  "f1147e9c73d95b8955fad12bcf7a1ecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualVisibles-request>)))
  "Returns full string definition for message of type '<SetVisualVisibles-request>"
  (cl:format cl:nil "string[] link_names~%string[] visual_names~%int8[] visibles~%bool block~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualVisibles-request)))
  "Returns full string definition for message of type 'SetVisualVisibles-request"
  (cl:format cl:nil "string[] link_names~%string[] visual_names~%int8[] visibles~%bool block~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualVisibles-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'link_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'visual_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'visibles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualVisibles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualVisibles-request
    (cl:cons ':link_names (link_names msg))
    (cl:cons ':visual_names (visual_names msg))
    (cl:cons ':visibles (visibles msg))
    (cl:cons ':block (block msg))
))
;//! \htmlinclude SetVisualVisibles-response.msg.html

(cl:defclass <SetVisualVisibles-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status_message
    :reader status_message
    :initarg :status_message
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (messages
    :reader messages
    :initarg :messages
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass SetVisualVisibles-response (<SetVisualVisibles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVisualVisibles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVisualVisibles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<SetVisualVisibles-response> is deprecated: use deepracer_msgs-srv:SetVisualVisibles-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetVisualVisibles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <SetVisualVisibles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetVisualVisibles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status-val is deprecated.  Use deepracer_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'messages-val :lambda-list '(m))
(cl:defmethod messages-val ((m <SetVisualVisibles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:messages-val is deprecated.  Use deepracer_msgs-srv:messages instead.")
  (messages m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVisualVisibles-response>) ostream)
  "Serializes a message object of type '<SetVisualVisibles-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status_message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status_message))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'status))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'messages))))
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
   (cl:slot-value msg 'messages))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVisualVisibles-response>) istream)
  "Deserializes a message object of type '<SetVisualVisibles-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status_message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status_message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'status) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'status)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'messages) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'messages)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVisualVisibles-response>)))
  "Returns string type for a service object of type '<SetVisualVisibles-response>"
  "deepracer_msgs/SetVisualVisiblesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualVisibles-response)))
  "Returns string type for a service object of type 'SetVisualVisibles-response"
  "deepracer_msgs/SetVisualVisiblesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVisualVisibles-response>)))
  "Returns md5sum for a message object of type '<SetVisualVisibles-response>"
  "f1147e9c73d95b8955fad12bcf7a1ecd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVisualVisibles-response)))
  "Returns md5sum for a message object of type 'SetVisualVisibles-response"
  "f1147e9c73d95b8955fad12bcf7a1ecd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVisualVisibles-response>)))
  "Returns full string definition for message of type '<SetVisualVisibles-response>"
  (cl:format cl:nil "bool success~%string status_message~%int8[] status~%string[] messages~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVisualVisibles-response)))
  "Returns full string definition for message of type 'SetVisualVisibles-response"
  (cl:format cl:nil "bool success~%string status_message~%int8[] status~%string[] messages~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVisualVisibles-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status_message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'status) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'messages) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVisualVisibles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVisualVisibles-response
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
    (cl:cons ':status (status msg))
    (cl:cons ':messages (messages msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVisualVisibles)))
  'SetVisualVisibles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVisualVisibles)))
  'SetVisualVisibles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVisualVisibles)))
  "Returns string type for a service object of type '<SetVisualVisibles>"
  "deepracer_msgs/SetVisualVisibles")