;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualTransparency)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualTransparency (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCY")
  (make-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCY"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCYREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCYREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCYRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCYRESPONSE"))

(in-package "ROS")





(defclass deepracer_msgs::SetVisualTransparencyRequest
  :super ros::object
  :slots (_link_name _visual_name _transparency _block ))

(defmethod deepracer_msgs::SetVisualTransparencyRequest
  (:init
   (&key
    ((:link_name __link_name) "")
    ((:visual_name __visual_name) "")
    ((:transparency __transparency) 0.0)
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_name (string __link_name))
   (setq _visual_name (string __visual_name))
   (setq _transparency (float __transparency))
   (setq _block __block)
   self)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:visual_name
   (&optional __visual_name)
   (if __visual_name (setq _visual_name __visual_name)) _visual_name)
  (:transparency
   (&optional __transparency)
   (if __transparency (setq _transparency __transparency)) _transparency)
  (:block
   (&optional __block)
   (if __block (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; string _link_name
    4 (length _link_name)
    ;; string _visual_name
    4 (length _visual_name)
    ;; float64 _transparency
    8
    ;; bool _block
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _link_name
       (write-long (length _link_name) s) (princ _link_name s)
     ;; string _visual_name
       (write-long (length _visual_name) s) (princ _visual_name s)
     ;; float64 _transparency
       (sys::poke _transparency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _link_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _visual_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _visual_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _transparency
     (setq _transparency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualTransparencyResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod deepracer_msgs::SetVisualTransparencyResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualTransparency
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualTransparency :md5sum-) "8d620635331bc4411a9425df4f8058e7")
(setf (get deepracer_msgs::SetVisualTransparency :datatype-) "deepracer_msgs/SetVisualTransparency")
(setf (get deepracer_msgs::SetVisualTransparency :request) deepracer_msgs::SetVisualTransparencyRequest)
(setf (get deepracer_msgs::SetVisualTransparency :response) deepracer_msgs::SetVisualTransparencyResponse)

(defmethod deepracer_msgs::SetVisualTransparencyRequest
  (:response () (instance deepracer_msgs::SetVisualTransparencyResponse :init)))

(setf (get deepracer_msgs::SetVisualTransparencyRequest :md5sum-) "8d620635331bc4411a9425df4f8058e7")
(setf (get deepracer_msgs::SetVisualTransparencyRequest :datatype-) "deepracer_msgs/SetVisualTransparencyRequest")
(setf (get deepracer_msgs::SetVisualTransparencyRequest :definition-)
      "string link_name
string visual_name
float64 transparency
bool block
---
bool success
string status_message
")

(setf (get deepracer_msgs::SetVisualTransparencyResponse :md5sum-) "8d620635331bc4411a9425df4f8058e7")
(setf (get deepracer_msgs::SetVisualTransparencyResponse :datatype-) "deepracer_msgs/SetVisualTransparencyResponse")
(setf (get deepracer_msgs::SetVisualTransparencyResponse :definition-)
      "string link_name
string visual_name
float64 transparency
bool block
---
bool success
string status_message
")



(provide :deepracer_msgs/SetVisualTransparency "8d620635331bc4411a9425df4f8058e7")


