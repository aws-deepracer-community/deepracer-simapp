; Auto-generated. Do not edit!


(cl:in-package robomaker_simulation_msgs-srv)


;//! \htmlinclude ListTags-request.msg.html

(cl:defclass <ListTags-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ListTags-request (<ListTags-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListTags-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListTags-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-srv:<ListTags-request> is deprecated: use robomaker_simulation_msgs-srv:ListTags-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListTags-request>) ostream)
  "Serializes a message object of type '<ListTags-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListTags-request>) istream)
  "Deserializes a message object of type '<ListTags-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListTags-request>)))
  "Returns string type for a service object of type '<ListTags-request>"
  "robomaker_simulation_msgs/ListTagsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListTags-request)))
  "Returns string type for a service object of type 'ListTags-request"
  "robomaker_simulation_msgs/ListTagsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListTags-request>)))
  "Returns md5sum for a message object of type '<ListTags-request>"
  "f351af60fa94133527414ffdcaacf51b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListTags-request)))
  "Returns md5sum for a message object of type 'ListTags-request"
  "f351af60fa94133527414ffdcaacf51b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListTags-request>)))
  "Returns full string definition for message of type '<ListTags-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListTags-request)))
  "Returns full string definition for message of type 'ListTags-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListTags-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListTags-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ListTags-request
))
;//! \htmlinclude ListTags-response.msg.html

(cl:defclass <ListTags-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (tags
    :reader tags
    :initarg :tags
    :type (cl:vector robomaker_simulation_msgs-msg:Tag)
   :initform (cl:make-array 0 :element-type 'robomaker_simulation_msgs-msg:Tag :initial-element (cl:make-instance 'robomaker_simulation_msgs-msg:Tag))))
)

(cl:defclass ListTags-response (<ListTags-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListTags-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListTags-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-srv:<ListTags-response> is deprecated: use robomaker_simulation_msgs-srv:ListTags-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ListTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:success-val is deprecated.  Use robomaker_simulation_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ListTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:message-val is deprecated.  Use robomaker_simulation_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'tags-val :lambda-list '(m))
(cl:defmethod tags-val ((m <ListTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:tags-val is deprecated.  Use robomaker_simulation_msgs-srv:tags instead.")
  (tags m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListTags-response>) ostream)
  "Serializes a message object of type '<ListTags-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tags))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tags))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListTags-response>) istream)
  "Deserializes a message object of type '<ListTags-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tags) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tags)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robomaker_simulation_msgs-msg:Tag))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListTags-response>)))
  "Returns string type for a service object of type '<ListTags-response>"
  "robomaker_simulation_msgs/ListTagsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListTags-response)))
  "Returns string type for a service object of type 'ListTags-response"
  "robomaker_simulation_msgs/ListTagsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListTags-response>)))
  "Returns md5sum for a message object of type '<ListTags-response>"
  "f351af60fa94133527414ffdcaacf51b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListTags-response)))
  "Returns md5sum for a message object of type 'ListTags-response"
  "f351af60fa94133527414ffdcaacf51b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListTags-response>)))
  "Returns full string definition for message of type '<ListTags-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%string message~%~%~%~%Tag[] tags~%~%================================================================================~%MSG: robomaker_simulation_msgs/Tag~%# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.~%# SPDX-License-Identifier: Apache-2.0~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListTags-response)))
  "Returns full string definition for message of type 'ListTags-response"
  (cl:format cl:nil "~%bool success~%~%~%~%string message~%~%~%~%Tag[] tags~%~%================================================================================~%MSG: robomaker_simulation_msgs/Tag~%# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.~%# SPDX-License-Identifier: Apache-2.0~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListTags-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tags) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListTags-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ListTags-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':tags (tags msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ListTags)))
  'ListTags-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ListTags)))
  'ListTags-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListTags)))
  "Returns string type for a service object of type '<ListTags>"
  "robomaker_simulation_msgs/ListTags")