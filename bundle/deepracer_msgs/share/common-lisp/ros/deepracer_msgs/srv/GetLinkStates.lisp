; Auto-generated. Do not edit!


(cl:in-package deepracer_msgs-srv)


;//! \htmlinclude GetLinkStates-request.msg.html

(cl:defclass <GetLinkStates-request> (roslisp-msg-protocol:ros-message)
  ((link_names
    :reader link_names
    :initarg :link_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (reference_frames
    :reader reference_frames
    :initarg :reference_frames
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetLinkStates-request (<GetLinkStates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLinkStates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLinkStates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<GetLinkStates-request> is deprecated: use deepracer_msgs-srv:GetLinkStates-request instead.")))

(cl:ensure-generic-function 'link_names-val :lambda-list '(m))
(cl:defmethod link_names-val ((m <GetLinkStates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_names-val is deprecated.  Use deepracer_msgs-srv:link_names instead.")
  (link_names m))

(cl:ensure-generic-function 'reference_frames-val :lambda-list '(m))
(cl:defmethod reference_frames-val ((m <GetLinkStates-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:reference_frames-val is deprecated.  Use deepracer_msgs-srv:reference_frames instead.")
  (reference_frames m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLinkStates-request>) ostream)
  "Serializes a message object of type '<GetLinkStates-request>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'reference_frames))))
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
   (cl:slot-value msg 'reference_frames))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLinkStates-request>) istream)
  "Deserializes a message object of type '<GetLinkStates-request>"
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
  (cl:setf (cl:slot-value msg 'reference_frames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'reference_frames)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLinkStates-request>)))
  "Returns string type for a service object of type '<GetLinkStates-request>"
  "deepracer_msgs/GetLinkStatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLinkStates-request)))
  "Returns string type for a service object of type 'GetLinkStates-request"
  "deepracer_msgs/GetLinkStatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLinkStates-request>)))
  "Returns md5sum for a message object of type '<GetLinkStates-request>"
  "1bc3add906c1ea529af2b9b4ae208741")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLinkStates-request)))
  "Returns md5sum for a message object of type 'GetLinkStates-request"
  "1bc3add906c1ea529af2b9b4ae208741")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLinkStates-request>)))
  "Returns full string definition for message of type '<GetLinkStates-request>"
  (cl:format cl:nil "string[] link_names~%~%string[] reference_frames~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLinkStates-request)))
  "Returns full string definition for message of type 'GetLinkStates-request"
  (cl:format cl:nil "string[] link_names~%~%string[] reference_frames~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLinkStates-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'link_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'reference_frames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLinkStates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLinkStates-request
    (cl:cons ':link_names (link_names msg))
    (cl:cons ':reference_frames (reference_frames msg))
))
;//! \htmlinclude GetLinkStates-response.msg.html

(cl:defclass <GetLinkStates-response> (roslisp-msg-protocol:ros-message)
  ((link_states
    :reader link_states
    :initarg :link_states
    :type (cl:vector gazebo_msgs-msg:LinkState)
   :initform (cl:make-array 0 :element-type 'gazebo_msgs-msg:LinkState :initial-element (cl:make-instance 'gazebo_msgs-msg:LinkState)))
   (success
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

(cl:defclass GetLinkStates-response (<GetLinkStates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLinkStates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLinkStates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_msgs-srv:<GetLinkStates-response> is deprecated: use deepracer_msgs-srv:GetLinkStates-response instead.")))

(cl:ensure-generic-function 'link_states-val :lambda-list '(m))
(cl:defmethod link_states-val ((m <GetLinkStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:link_states-val is deprecated.  Use deepracer_msgs-srv:link_states instead.")
  (link_states m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetLinkStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:success-val is deprecated.  Use deepracer_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status_message-val :lambda-list '(m))
(cl:defmethod status_message-val ((m <GetLinkStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status_message-val is deprecated.  Use deepracer_msgs-srv:status_message instead.")
  (status_message m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GetLinkStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:status-val is deprecated.  Use deepracer_msgs-srv:status instead.")
  (status m))

(cl:ensure-generic-function 'messages-val :lambda-list '(m))
(cl:defmethod messages-val ((m <GetLinkStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_msgs-srv:messages-val is deprecated.  Use deepracer_msgs-srv:messages instead.")
  (messages m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLinkStates-response>) ostream)
  "Serializes a message object of type '<GetLinkStates-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'link_states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'link_states))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLinkStates-response>) istream)
  "Deserializes a message object of type '<GetLinkStates-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'link_states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'link_states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'gazebo_msgs-msg:LinkState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLinkStates-response>)))
  "Returns string type for a service object of type '<GetLinkStates-response>"
  "deepracer_msgs/GetLinkStatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLinkStates-response)))
  "Returns string type for a service object of type 'GetLinkStates-response"
  "deepracer_msgs/GetLinkStatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLinkStates-response>)))
  "Returns md5sum for a message object of type '<GetLinkStates-response>"
  "1bc3add906c1ea529af2b9b4ae208741")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLinkStates-response)))
  "Returns md5sum for a message object of type 'GetLinkStates-response"
  "1bc3add906c1ea529af2b9b4ae208741")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLinkStates-response>)))
  "Returns full string definition for message of type '<GetLinkStates-response>"
  (cl:format cl:nil "gazebo_msgs/LinkState[] link_states~%bool success~%string status_message~%int8[] status~%string[] messages~%~%================================================================================~%MSG: gazebo_msgs/LinkState~%# @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.~%string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]~%geometry_msgs/Pose pose     # desired pose in reference frame~%geometry_msgs/Twist twist   # desired twist in reference frame~%string reference_frame      # set pose/twist relative to the frame of this link/body~%                            # leave empty or \"world\" or \"map\" defaults to world-frame~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLinkStates-response)))
  "Returns full string definition for message of type 'GetLinkStates-response"
  (cl:format cl:nil "gazebo_msgs/LinkState[] link_states~%bool success~%string status_message~%int8[] status~%string[] messages~%~%================================================================================~%MSG: gazebo_msgs/LinkState~%# @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.~%string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]~%geometry_msgs/Pose pose     # desired pose in reference frame~%geometry_msgs/Twist twist   # desired twist in reference frame~%string reference_frame      # set pose/twist relative to the frame of this link/body~%                            # leave empty or \"world\" or \"map\" defaults to world-frame~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLinkStates-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'link_states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     4 (cl:length (cl:slot-value msg 'status_message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'status) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'messages) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLinkStates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLinkStates-response
    (cl:cons ':link_states (link_states msg))
    (cl:cons ':success (success msg))
    (cl:cons ':status_message (status_message msg))
    (cl:cons ':status (status msg))
    (cl:cons ':messages (messages msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetLinkStates)))
  'GetLinkStates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetLinkStates)))
  'GetLinkStates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLinkStates)))
  "Returns string type for a service object of type '<GetLinkStates>"
  "deepracer_msgs/GetLinkStates")