; Auto-generated. Do not edit!


(cl:in-package robomaker_simulation_msgs-srv)


;//! \htmlinclude AddTags-request.msg.html

(cl:defclass <AddTags-request> (roslisp-msg-protocol:ros-message)
  ((tags
    :reader tags
    :initarg :tags
    :type (cl:vector robomaker_simulation_msgs-msg:Tag)
   :initform (cl:make-array 0 :element-type 'robomaker_simulation_msgs-msg:Tag :initial-element (cl:make-instance 'robomaker_simulation_msgs-msg:Tag))))
)

(cl:defclass AddTags-request (<AddTags-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTags-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTags-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-srv:<AddTags-request> is deprecated: use robomaker_simulation_msgs-srv:AddTags-request instead.")))

(cl:ensure-generic-function 'tags-val :lambda-list '(m))
(cl:defmethod tags-val ((m <AddTags-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:tags-val is deprecated.  Use robomaker_simulation_msgs-srv:tags instead.")
  (tags m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTags-request>) ostream)
  "Serializes a message object of type '<AddTags-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tags))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tags))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTags-request>) istream)
  "Deserializes a message object of type '<AddTags-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTags-request>)))
  "Returns string type for a service object of type '<AddTags-request>"
  "robomaker_simulation_msgs/AddTagsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTags-request)))
  "Returns string type for a service object of type 'AddTags-request"
  "robomaker_simulation_msgs/AddTagsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTags-request>)))
  "Returns md5sum for a message object of type '<AddTags-request>"
  "637781bbb7edf1a52e9fe238f1ba7d6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTags-request)))
  "Returns md5sum for a message object of type 'AddTags-request"
  "637781bbb7edf1a52e9fe238f1ba7d6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTags-request>)))
  "Returns full string definition for message of type '<AddTags-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%Tag[] tags~%~%================================================================================~%MSG: robomaker_simulation_msgs/Tag~%# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.~%# SPDX-License-Identifier: Apache-2.0~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTags-request)))
  "Returns full string definition for message of type 'AddTags-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%Tag[] tags~%~%================================================================================~%MSG: robomaker_simulation_msgs/Tag~%# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.~%# SPDX-License-Identifier: Apache-2.0~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTags-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tags) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTags-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTags-request
    (cl:cons ':tags (tags msg))
))
;//! \htmlinclude AddTags-response.msg.html

(cl:defclass <AddTags-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass AddTags-response (<AddTags-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddTags-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddTags-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robomaker_simulation_msgs-srv:<AddTags-response> is deprecated: use robomaker_simulation_msgs-srv:AddTags-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AddTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:success-val is deprecated.  Use robomaker_simulation_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <AddTags-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robomaker_simulation_msgs-srv:message-val is deprecated.  Use robomaker_simulation_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddTags-response>) ostream)
  "Serializes a message object of type '<AddTags-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddTags-response>) istream)
  "Deserializes a message object of type '<AddTags-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddTags-response>)))
  "Returns string type for a service object of type '<AddTags-response>"
  "robomaker_simulation_msgs/AddTagsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTags-response)))
  "Returns string type for a service object of type 'AddTags-response"
  "robomaker_simulation_msgs/AddTagsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddTags-response>)))
  "Returns md5sum for a message object of type '<AddTags-response>"
  "637781bbb7edf1a52e9fe238f1ba7d6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddTags-response)))
  "Returns md5sum for a message object of type 'AddTags-response"
  "637781bbb7edf1a52e9fe238f1ba7d6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddTags-response>)))
  "Returns full string definition for message of type '<AddTags-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddTags-response)))
  "Returns full string definition for message of type 'AddTags-response"
  (cl:format cl:nil "~%bool success~%~%~%~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddTags-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddTags-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddTags-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddTags)))
  'AddTags-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddTags)))
  'AddTags-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddTags)))
  "Returns string type for a service object of type '<AddTags>"
  "robomaker_simulation_msgs/AddTags")