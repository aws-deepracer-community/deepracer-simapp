;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::GetLightNames)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'GetLightNames (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::GETLIGHTNAMES")
  (make-package "DEEPRACER_MSGS::GETLIGHTNAMES"))
(unless (find-package "DEEPRACER_MSGS::GETLIGHTNAMESREQUEST")
  (make-package "DEEPRACER_MSGS::GETLIGHTNAMESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::GETLIGHTNAMESRESPONSE")
  (make-package "DEEPRACER_MSGS::GETLIGHTNAMESRESPONSE"))

(in-package "ROS")





(defclass deepracer_msgs::GetLightNamesRequest
  :super ros::object
  :slots ())

(defmethod deepracer_msgs::GetLightNamesRequest
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

(defclass deepracer_msgs::GetLightNamesResponse
  :super ros::object
  :slots (_light_names _success _status_message ))

(defmethod deepracer_msgs::GetLightNamesResponse
  (:init
   (&key
    ((:light_names __light_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _light_names __light_names)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:light_names
   (&optional __light_names)
   (if __light_names (setq _light_names __light_names)) _light_names)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; string[] _light_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _light_names)) 4
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _light_names
     (write-long (length _light_names) s)
     (dolist (elem _light_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _light_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _light_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _light_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_msgs::GetLightNames
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::GetLightNames :md5sum-) "6dff1e71fe0fb6f3d275724e4c746d5e")
(setf (get deepracer_msgs::GetLightNames :datatype-) "deepracer_msgs/GetLightNames")
(setf (get deepracer_msgs::GetLightNames :request) deepracer_msgs::GetLightNamesRequest)
(setf (get deepracer_msgs::GetLightNames :response) deepracer_msgs::GetLightNamesResponse)

(defmethod deepracer_msgs::GetLightNamesRequest
  (:response () (instance deepracer_msgs::GetLightNamesResponse :init)))

(setf (get deepracer_msgs::GetLightNamesRequest :md5sum-) "6dff1e71fe0fb6f3d275724e4c746d5e")
(setf (get deepracer_msgs::GetLightNamesRequest :datatype-) "deepracer_msgs/GetLightNamesRequest")
(setf (get deepracer_msgs::GetLightNamesRequest :definition-)
      "---
string[] light_names
bool success
string status_message
")

(setf (get deepracer_msgs::GetLightNamesResponse :md5sum-) "6dff1e71fe0fb6f3d275724e4c746d5e")
(setf (get deepracer_msgs::GetLightNamesResponse :datatype-) "deepracer_msgs/GetLightNamesResponse")
(setf (get deepracer_msgs::GetLightNamesResponse :definition-)
      "---
string[] light_names
bool success
string status_message
")



(provide :deepracer_msgs/GetLightNames "6dff1e71fe0fb6f3d275724e4c746d5e")


