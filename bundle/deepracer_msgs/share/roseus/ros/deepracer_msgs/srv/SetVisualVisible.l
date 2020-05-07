;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualVisible)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualVisible (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALVISIBLE")
  (make-package "DEEPRACER_MSGS::SETVISUALVISIBLE"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALVISIBLEREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALVISIBLEREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALVISIBLERESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALVISIBLERESPONSE"))

(in-package "ROS")





(defclass deepracer_msgs::SetVisualVisibleRequest
  :super ros::object
  :slots (_link_name _visual_name _visible _block ))

(defmethod deepracer_msgs::SetVisualVisibleRequest
  (:init
   (&key
    ((:link_name __link_name) "")
    ((:visual_name __visual_name) "")
    ((:visible __visible) nil)
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_name (string __link_name))
   (setq _visual_name (string __visual_name))
   (setq _visible __visible)
   (setq _block __block)
   self)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:visual_name
   (&optional __visual_name)
   (if __visual_name (setq _visual_name __visual_name)) _visual_name)
  (:visible
   (&optional __visible)
   (if __visible (setq _visible __visible)) _visible)
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
    ;; bool _visible
    1
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
     ;; bool _visible
       (if _visible (write-byte -1 s) (write-byte 0 s))
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
   ;; bool _visible
     (setq _visible (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualVisibleResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod deepracer_msgs::SetVisualVisibleResponse
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

(defclass deepracer_msgs::SetVisualVisible
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualVisible :md5sum-) "91b990c22664a5d31142982439721968")
(setf (get deepracer_msgs::SetVisualVisible :datatype-) "deepracer_msgs/SetVisualVisible")
(setf (get deepracer_msgs::SetVisualVisible :request) deepracer_msgs::SetVisualVisibleRequest)
(setf (get deepracer_msgs::SetVisualVisible :response) deepracer_msgs::SetVisualVisibleResponse)

(defmethod deepracer_msgs::SetVisualVisibleRequest
  (:response () (instance deepracer_msgs::SetVisualVisibleResponse :init)))

(setf (get deepracer_msgs::SetVisualVisibleRequest :md5sum-) "91b990c22664a5d31142982439721968")
(setf (get deepracer_msgs::SetVisualVisibleRequest :datatype-) "deepracer_msgs/SetVisualVisibleRequest")
(setf (get deepracer_msgs::SetVisualVisibleRequest :definition-)
      "string link_name
string visual_name
bool visible
bool block
---
bool success
string status_message
")

(setf (get deepracer_msgs::SetVisualVisibleResponse :md5sum-) "91b990c22664a5d31142982439721968")
(setf (get deepracer_msgs::SetVisualVisibleResponse :datatype-) "deepracer_msgs/SetVisualVisibleResponse")
(setf (get deepracer_msgs::SetVisualVisibleResponse :definition-)
      "string link_name
string visual_name
bool visible
bool block
---
bool success
string status_message
")



(provide :deepracer_msgs/SetVisualVisible "91b990c22664a5d31142982439721968")


