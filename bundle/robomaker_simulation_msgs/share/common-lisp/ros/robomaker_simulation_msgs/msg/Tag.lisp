; Auto-generated. Do not edit!


(cl:in-package robomaker_simulation_msgs-msg)


;//! \htmlinclude Tag.msg.html

(cl:defclass <Tag> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass Tag (<Tag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-msg:<Tag> is deprecated: use robomaker_simulation_msgs-msg:Tag instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <Tag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-msg:key-val is deprecated.  Use robomaker_simulation_msgs-msg:key instead.")
  (key m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Tag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-msg:value-val is deprecated.  Use robomaker_simulation_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tag>) ostream)
  "Serializes a message object of type '<Tag>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'key))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'key))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tag>) istream)
  "Deserializes a message object of type '<Tag>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'key) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tag>)))
  "Returns string type for a message object of type '<Tag>"
  "robomaker_simulation_msgs/Tag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tag)))
  "Returns string type for a message object of type 'Tag"
  "robomaker_simulation_msgs/Tag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tag>)))
  "Returns md5sum for a message object of type '<Tag>"
  "cf57fdc6617a881a88c16e768132149c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tag)))
  "Returns md5sum for a message object of type 'Tag"
  "cf57fdc6617a881a88c16e768132149c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tag>)))
  "Returns full string definition for message of type '<Tag>"
  (cl:format cl:nil "# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.~%# SPDX-License-Identifier: Apache-2.0~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tag)))
  "Returns full string definition for message of type 'Tag"
  (cl:format cl:nil "# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.~%# SPDX-License-Identifier: Apache-2.0~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tag>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'key))
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tag>))
  "Converts a ROS message object to a list"
  (cl:list 'Tag
    (cl:cons ':key (key msg))
    (cl:cons ':value (value msg))
))
