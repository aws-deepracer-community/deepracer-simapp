;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualTransparencies)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualTransparencies (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCIES")
  (make-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCIES"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCIESREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCIESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCIESRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALTRANSPARENCIESRESPONSE"))

(in-package "ROS")





(defclass deepracer_msgs::SetVisualTransparenciesRequest
  :super ros::object
  :slots (_link_names _visual_names _transparencies _block ))

(defmethod deepracer_msgs::SetVisualTransparenciesRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:transparencies __transparencies) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   (setq _transparencies __transparencies)
   (setq _block __block)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:transparencies
   (&optional __transparencies)
   (if __transparencies (setq _transparencies __transparencies)) _transparencies)
  (:block
   (&optional __block)
   (if __block (setq _block __block)) _block)
  (:serialization-length
   ()
   (+
    ;; string[] _link_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _link_names)) 4
    ;; string[] _visual_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _visual_names)) 4
    ;; float64[] _transparencies
    (* 8    (length _transparencies)) 4
    ;; bool _block
    1
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
     ;; string[] _visual_names
     (write-long (length _visual_names) s)
     (dolist (elem _visual_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _transparencies
     (write-long (length _transparencies) s)
     (dotimes (i (length _transparencies))
       (sys::poke (elt _transparencies i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _block
       (if _block (write-byte -1 s) (write-byte 0 s))
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
   ;; string[] _visual_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _visual_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _visual_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _transparencies
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _transparencies (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _transparencies i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualTransparenciesResponse
  :super ros::object
  :slots (_success _status_message _status _messages ))

(defmethod deepracer_msgs::SetVisualTransparenciesResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    ((:status __status) (make-array 0 :initial-element 0 :element-type :integer))
    ((:messages __messages) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   (setq _status __status)
   (setq _messages __messages)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:messages
   (&optional __messages)
   (if __messages (setq _messages __messages)) _messages)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ;; int8[] _status
    (* 1    (length _status)) 4
    ;; string[] _messages
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _messages)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;; int8[] _status
     (write-long (length _status) s)
     (dotimes (i (length _status))
       (write-byte (elt _status i) s)
       )
     ;; string[] _messages
     (write-long (length _messages) s)
     (dolist (elem _messages)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8[] _status
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _status (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _status i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _status i) 127) (setf (elt _status i) (- (elt _status i) 256)))
     ))
   ;; string[] _messages
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _messages (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _messages i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualTransparencies
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualTransparencies :md5sum-) "34fbd02ec7c4149646080e935ee8f1b0")
(setf (get deepracer_msgs::SetVisualTransparencies :datatype-) "deepracer_msgs/SetVisualTransparencies")
(setf (get deepracer_msgs::SetVisualTransparencies :request) deepracer_msgs::SetVisualTransparenciesRequest)
(setf (get deepracer_msgs::SetVisualTransparencies :response) deepracer_msgs::SetVisualTransparenciesResponse)

(defmethod deepracer_msgs::SetVisualTransparenciesRequest
  (:response () (instance deepracer_msgs::SetVisualTransparenciesResponse :init)))

(setf (get deepracer_msgs::SetVisualTransparenciesRequest :md5sum-) "34fbd02ec7c4149646080e935ee8f1b0")
(setf (get deepracer_msgs::SetVisualTransparenciesRequest :datatype-) "deepracer_msgs/SetVisualTransparenciesRequest")
(setf (get deepracer_msgs::SetVisualTransparenciesRequest :definition-)
      "string[] link_names
string[] visual_names
float64[] transparencies
bool block
---
bool success
string status_message
int8[] status
string[] messages
")

(setf (get deepracer_msgs::SetVisualTransparenciesResponse :md5sum-) "34fbd02ec7c4149646080e935ee8f1b0")
(setf (get deepracer_msgs::SetVisualTransparenciesResponse :datatype-) "deepracer_msgs/SetVisualTransparenciesResponse")
(setf (get deepracer_msgs::SetVisualTransparenciesResponse :definition-)
      "string[] link_names
string[] visual_names
float64[] transparencies
bool block
---
bool success
string status_message
int8[] status
string[] messages
")



(provide :deepracer_msgs/SetVisualTransparencies "34fbd02ec7c4149646080e935ee8f1b0")


