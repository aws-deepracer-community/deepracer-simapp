; Auto-generated. Do not edit!


(cl:in-package deepracer_simulation_environment-srv)


;//! \htmlinclude TopCamDataSrv-request.msg.html

(cl:defclass <TopCamDataSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TopCamDataSrv-request (<TopCamDataSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopCamDataSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopCamDataSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation_environment-srv:<TopCamDataSrv-request> is deprecated: use deepracer_simulation_environment-srv:TopCamDataSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopCamDataSrv-request>) ostream)
  "Serializes a message object of type '<TopCamDataSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopCamDataSrv-request>) istream)
  "Deserializes a message object of type '<TopCamDataSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopCamDataSrv-request>)))
  "Returns string type for a service object of type '<TopCamDataSrv-request>"
  "deepracer_simulation_environment/TopCamDataSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopCamDataSrv-request)))
  "Returns string type for a service object of type 'TopCamDataSrv-request"
  "deepracer_simulation_environment/TopCamDataSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopCamDataSrv-request>)))
  "Returns md5sum for a message object of type '<TopCamDataSrv-request>"
  "eed9195ebff712c1941314f098f70944")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopCamDataSrv-request)))
  "Returns md5sum for a message object of type 'TopCamDataSrv-request"
  "eed9195ebff712c1941314f098f70944")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopCamDataSrv-request>)))
  "Returns full string definition for message of type '<TopCamDataSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopCamDataSrv-request)))
  "Returns full string definition for message of type 'TopCamDataSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopCamDataSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopCamDataSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TopCamDataSrv-request
))
;//! \htmlinclude TopCamDataSrv-response.msg.html

(cl:defclass <TopCamDataSrv-response> (roslisp-msg-protocol:ros-message)
  ((horizontal_fov
    :reader horizontal_fov
    :initarg :horizontal_fov
    :type cl:float
    :initform 0.0)
   (padding_pct
    :reader padding_pct
    :initarg :padding_pct
    :type cl:float
    :initform 0.0)
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:float
    :initform 0.0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:float
    :initform 0.0))
)

(cl:defclass TopCamDataSrv-response (<TopCamDataSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopCamDataSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopCamDataSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation_environment-srv:<TopCamDataSrv-response> is deprecated: use deepracer_simulation_environment-srv:TopCamDataSrv-response instead.")))

(cl:ensure-generic-function 'horizontal_fov-val :lambda-list '(m))
(cl:defmethod horizontal_fov-val ((m <TopCamDataSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:horizontal_fov-val is deprecated.  Use deepracer_simulation_environment-srv:horizontal_fov instead.")
  (horizontal_fov m))

(cl:ensure-generic-function 'padding_pct-val :lambda-list '(m))
(cl:defmethod padding_pct-val ((m <TopCamDataSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:padding_pct-val is deprecated.  Use deepracer_simulation_environment-srv:padding_pct instead.")
  (padding_pct m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <TopCamDataSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:image_width-val is deprecated.  Use deepracer_simulation_environment-srv:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <TopCamDataSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation_environment-srv:image_height-val is deprecated.  Use deepracer_simulation_environment-srv:image_height instead.")
  (image_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopCamDataSrv-response>) ostream)
  "Serializes a message object of type '<TopCamDataSrv-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'horizontal_fov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'padding_pct))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'image_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'image_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopCamDataSrv-response>) istream)
  "Deserializes a message object of type '<TopCamDataSrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal_fov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'padding_pct) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'image_width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'image_height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopCamDataSrv-response>)))
  "Returns string type for a service object of type '<TopCamDataSrv-response>"
  "deepracer_simulation_environment/TopCamDataSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopCamDataSrv-response)))
  "Returns string type for a service object of type 'TopCamDataSrv-response"
  "deepracer_simulation_environment/TopCamDataSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopCamDataSrv-response>)))
  "Returns md5sum for a message object of type '<TopCamDataSrv-response>"
  "eed9195ebff712c1941314f098f70944")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopCamDataSrv-response)))
  "Returns md5sum for a message object of type 'TopCamDataSrv-response"
  "eed9195ebff712c1941314f098f70944")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopCamDataSrv-response>)))
  "Returns full string definition for message of type '<TopCamDataSrv-response>"
  (cl:format cl:nil "float32 horizontal_fov~%float32 padding_pct~%float32 image_width~%float32 image_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopCamDataSrv-response)))
  "Returns full string definition for message of type 'TopCamDataSrv-response"
  (cl:format cl:nil "float32 horizontal_fov~%float32 padding_pct~%float32 image_width~%float32 image_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopCamDataSrv-response>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopCamDataSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TopCamDataSrv-response
    (cl:cons ':horizontal_fov (horizontal_fov msg))
    (cl:cons ':padding_pct (padding_pct msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TopCamDataSrv)))
  'TopCamDataSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TopCamDataSrv)))
  'TopCamDataSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopCamDataSrv)))
  "Returns string type for a service object of type '<TopCamDataSrv>"
  "deepracer_simulation_environment/TopCamDataSrv")