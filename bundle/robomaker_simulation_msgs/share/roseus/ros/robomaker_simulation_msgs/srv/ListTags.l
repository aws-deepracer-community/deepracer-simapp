;; Auto-generated. Do not edit!


(when (boundp 'robomaker_simulation_msgs::ListTags)
  (if (not (find-package "ROBOMAKER_SIMULATION_MSGS"))
    (make-package "ROBOMAKER_SIMULATION_MSGS"))
  (shadow 'ListTags (find-package "ROBOMAKER_SIMULATION_MSGS")))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::LISTTAGS")
  (make-package "ROBOMAKER_SIMULATION_MSGS::LISTTAGS"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::LISTTAGSREQUEST")
  (make-package "ROBOMAKER_SIMULATION_MSGS::LISTTAGSREQUEST"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::LISTTAGSRESPONSE")
  (make-package "ROBOMAKER_SIMULATION_MSGS::LISTTAGSRESPONSE"))

(in-package "ROS")





(defclass robomaker_simulation_msgs::ListTagsRequest
  :super ros::object
  :slots ())

(defmethod robomaker_simulation_msgs::ListTagsRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass robomaker_simulation_msgs::ListTagsResponse
  :super ros::object
  :slots (_success _message _tags ))

(defmethod robomaker_simulation_msgs::ListTagsResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:tags __tags) (let (r) (dotimes (i 0) (push (instance robomaker_simulation_msgs::Tag :init) r)) r))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _tags __tags)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:tags
   (&rest __tags)
   (if (keywordp (car __tags))
       (send* _tags __tags)
     (progn
       (if __tags (setq _tags (car __tags)))
       _tags)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; robomaker_simulation_msgs/Tag[] _tags
    (apply #'+ (send-all _tags :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; robomaker_simulation_msgs/Tag[] _tags
     (write-long (length _tags) s)
     (dolist (elem _tags)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; robomaker_simulation_msgs/Tag[] _tags
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags (let (r) (dotimes (i n) (push (instance robomaker_simulation_msgs::Tag :init) r)) r))
     (dolist (elem- _tags)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass robomaker_simulation_msgs::ListTags
  :super ros::object
  :slots ())

(setf (get robomaker_simulation_msgs::ListTags :md5sum-) "f351af60fa94133527414ffdcaacf51b")
(setf (get robomaker_simulation_msgs::ListTags :datatype-) "robomaker_simulation_msgs/ListTags")
(setf (get robomaker_simulation_msgs::ListTags :request) robomaker_simulation_msgs::ListTagsRequest)
(setf (get robomaker_simulation_msgs::ListTags :response) robomaker_simulation_msgs::ListTagsResponse)

(defmethod robomaker_simulation_msgs::ListTagsRequest
  (:response () (instance robomaker_simulation_msgs::ListTagsResponse :init)))

(setf (get robomaker_simulation_msgs::ListTagsRequest :md5sum-) "f351af60fa94133527414ffdcaacf51b")
(setf (get robomaker_simulation_msgs::ListTagsRequest :datatype-) "robomaker_simulation_msgs/ListTagsRequest")
(setf (get robomaker_simulation_msgs::ListTagsRequest :definition-)
      "













---

bool success



string message



Tag[] tags

================================================================================
MSG: robomaker_simulation_msgs/Tag
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
string key
")

(setf (get robomaker_simulation_msgs::ListTagsResponse :md5sum-) "f351af60fa94133527414ffdcaacf51b")
(setf (get robomaker_simulation_msgs::ListTagsResponse :datatype-) "robomaker_simulation_msgs/ListTagsResponse")
(setf (get robomaker_simulation_msgs::ListTagsResponse :definition-)
      "













---

bool success



string message



Tag[] tags

================================================================================
MSG: robomaker_simulation_msgs/Tag
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
string key
")



(provide :robomaker_simulation_msgs/ListTags "f351af60fa94133527414ffdcaacf51b")


