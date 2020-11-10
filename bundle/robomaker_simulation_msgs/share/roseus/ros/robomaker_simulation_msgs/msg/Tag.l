;; Auto-generated. Do not edit!


(when (boundp 'robomaker_simulation_msgs::Tag)
  (if (not (find-package "ROBOMAKER_SIMULATION_MSGS"))
    (make-package "ROBOMAKER_SIMULATION_MSGS"))
  (shadow 'Tag (find-package "ROBOMAKER_SIMULATION_MSGS")))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::TAG")
  (make-package "ROBOMAKER_SIMULATION_MSGS::TAG"))

(in-package "ROS")
;;//! \htmlinclude Tag.msg.html


(defclass robomaker_simulation_msgs::Tag
  :super ros::object
  :slots (_key _value ))

(defmethod robomaker_simulation_msgs::Tag
  (:init
   (&key
    ((:key __key) "")
    ((:value __value) "")
    )
   (send-super :init)
   (setq _key (string __key))
   (setq _value (string __value))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _key
    4 (length _key)
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _key
       (write-long (length _key) s) (princ _key s)
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robomaker_simulation_msgs::Tag :md5sum-) "cf57fdc6617a881a88c16e768132149c")
(setf (get robomaker_simulation_msgs::Tag :datatype-) "robomaker_simulation_msgs/Tag")
(setf (get robomaker_simulation_msgs::Tag :definition-)
      "# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
string key
string value
")



(provide :robomaker_simulation_msgs/Tag "cf57fdc6617a881a88c16e768132149c")


