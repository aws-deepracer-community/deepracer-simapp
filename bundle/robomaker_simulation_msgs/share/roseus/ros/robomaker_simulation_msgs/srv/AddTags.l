;; Auto-generated. Do not edit!


(when (boundp 'robomaker_simulation_msgs::AddTags)
  (if (not (find-package "ROBOMAKER_SIMULATION_MSGS"))
    (make-package "ROBOMAKER_SIMULATION_MSGS"))
  (shadow 'AddTags (find-package "ROBOMAKER_SIMULATION_MSGS")))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::ADDTAGS")
  (make-package "ROBOMAKER_SIMULATION_MSGS::ADDTAGS"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::ADDTAGSREQUEST")
  (make-package "ROBOMAKER_SIMULATION_MSGS::ADDTAGSREQUEST"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::ADDTAGSRESPONSE")
  (make-package "ROBOMAKER_SIMULATION_MSGS::ADDTAGSRESPONSE"))

(in-package "ROS")





(defclass robomaker_simulation_msgs::AddTagsRequest
  :super ros::object
  :slots (_tags ))

(defmethod robomaker_simulation_msgs::AddTagsRequest
  (:init
   (&key
    ((:tags __tags) (let (r) (dotimes (i 0) (push (instance robomaker_simulation_msgs::Tag :init) r)) r))
    )
   (send-super :init)
   (setq _tags __tags)
   self)
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
    ;; robomaker_simulation_msgs/Tag[] _tags
    (apply #'+ (send-all _tags :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robomaker_simulation_msgs/Tag[] _tags
     (write-long (length _tags) s)
     (dolist (elem _tags)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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

(defclass robomaker_simulation_msgs::AddTagsResponse
  :super ros::object
  :slots (_success _message ))

(defmethod robomaker_simulation_msgs::AddTagsResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robomaker_simulation_msgs::AddTags
  :super ros::object
  :slots ())

(setf (get robomaker_simulation_msgs::AddTags :md5sum-) "637781bbb7edf1a52e9fe238f1ba7d6b")
(setf (get robomaker_simulation_msgs::AddTags :datatype-) "robomaker_simulation_msgs/AddTags")
(setf (get robomaker_simulation_msgs::AddTags :request) robomaker_simulation_msgs::AddTagsRequest)
(setf (get robomaker_simulation_msgs::AddTags :response) robomaker_simulation_msgs::AddTagsResponse)

(defmethod robomaker_simulation_msgs::AddTagsRequest
  (:response () (instance robomaker_simulation_msgs::AddTagsResponse :init)))

(setf (get robomaker_simulation_msgs::AddTagsRequest :md5sum-) "637781bbb7edf1a52e9fe238f1ba7d6b")
(setf (get robomaker_simulation_msgs::AddTagsRequest :datatype-) "robomaker_simulation_msgs/AddTagsRequest")
(setf (get robomaker_simulation_msgs::AddTagsRequest :definition-)
      "














Tag[] tags

================================================================================
MSG: robomaker_simulation_msgs/Tag
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
string key
---

bool success



string message
")

(setf (get robomaker_simulation_msgs::AddTagsResponse :md5sum-) "637781bbb7edf1a52e9fe238f1ba7d6b")
(setf (get robomaker_simulation_msgs::AddTagsResponse :datatype-) "robomaker_simulation_msgs/AddTagsResponse")
(setf (get robomaker_simulation_msgs::AddTagsResponse :definition-)
      "














Tag[] tags

================================================================================
MSG: robomaker_simulation_msgs/Tag
# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
string key
---

bool success



string message
")



(provide :robomaker_simulation_msgs/AddTags "637781bbb7edf1a52e9fe238f1ba7d6b")


