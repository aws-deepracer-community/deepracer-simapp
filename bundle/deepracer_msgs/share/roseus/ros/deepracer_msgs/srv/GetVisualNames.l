;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::GetVisualNames)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'GetVisualNames (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::GETVISUALNAMES")
  (make-package "DEEPRACER_MSGS::GETVISUALNAMES"))
(unless (find-package "DEEPRACER_MSGS::GETVISUALNAMESREQUEST")
  (make-package "DEEPRACER_MSGS::GETVISUALNAMESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::GETVISUALNAMESRESPONSE")
  (make-package "DEEPRACER_MSGS::GETVISUALNAMESRESPONSE"))

(in-package "ROS")





(defclass deepracer_msgs::GetVisualNamesRequest
  :super ros::object
  :slots (_link_names ))

(defmethod deepracer_msgs::GetVisualNamesRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _link_names __link_names)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _link_names
     (write-long (length _link_names) s)
     (dolist (elem _link_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _link_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _link_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass deepracer_msgs::GetVisualNamesResponse
  :super ros::object
  :slots (_visual_names _link_names _success _status_message ))

(defmethod deepracer_msgs::GetVisualNamesResponse
  (:init
   (&key
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _visual_names __visual_names)
   (setq _link_names __link_names)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; string[] _visual_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _visual_names)) 4
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _visual_names
     (write-long (length _visual_names) s)
     (dolist (elem _visual_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _link_names
     (write-long (length _link_names) s)
     (dolist (elem _link_names)
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
   ;; string[] _visual_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _visual_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _visual_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _link_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _link_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _link_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_msgs::GetVisualNames
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::GetVisualNames :md5sum-) "f43c94dfba3ba3ebdb7d26fb08b0edd1")
(setf (get deepracer_msgs::GetVisualNames :datatype-) "deepracer_msgs/GetVisualNames")
(setf (get deepracer_msgs::GetVisualNames :request) deepracer_msgs::GetVisualNamesRequest)
(setf (get deepracer_msgs::GetVisualNames :response) deepracer_msgs::GetVisualNamesResponse)

(defmethod deepracer_msgs::GetVisualNamesRequest
  (:response () (instance deepracer_msgs::GetVisualNamesResponse :init)))

(setf (get deepracer_msgs::GetVisualNamesRequest :md5sum-) "f43c94dfba3ba3ebdb7d26fb08b0edd1")
(setf (get deepracer_msgs::GetVisualNamesRequest :datatype-) "deepracer_msgs/GetVisualNamesRequest")
(setf (get deepracer_msgs::GetVisualNamesRequest :definition-)
      "string[] link_names
---
string[] visual_names
string[] link_names
bool success
string status_message
")

(setf (get deepracer_msgs::GetVisualNamesResponse :md5sum-) "f43c94dfba3ba3ebdb7d26fb08b0edd1")
(setf (get deepracer_msgs::GetVisualNamesResponse :datatype-) "deepracer_msgs/GetVisualNamesResponse")
(setf (get deepracer_msgs::GetVisualNamesResponse :definition-)
      "string[] link_names
---
string[] visual_names
string[] link_names
bool success
string status_message
")



(provide :deepracer_msgs/GetVisualNames "f43c94dfba3ba3ebdb7d26fb08b0edd1")


