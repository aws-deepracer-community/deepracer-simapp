;; Auto-generated. Do not edit!


(when (boundp 'deepracer_simulation_environment::VirtualEventVideoEditSrv)
  (if (not (find-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
    (make-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
  (shadow 'VirtualEventVideoEditSrv (find-package "DEEPRACER_SIMULATION_ENVIRONMENT")))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIRTUALEVENTVIDEOEDITSRV")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIRTUALEVENTVIDEOEDITSRV"))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIRTUALEVENTVIDEOEDITSRVREQUEST")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIRTUALEVENTVIDEOEDITSRVREQUEST"))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIRTUALEVENTVIDEOEDITSRVRESPONSE")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIRTUALEVENTVIDEOEDITSRVRESPONSE"))

(in-package "ROS")





(defclass deepracer_simulation_environment::VirtualEventVideoEditSrvRequest
  :super ros::object
  :slots (_display_name _racecar_color ))

(defmethod deepracer_simulation_environment::VirtualEventVideoEditSrvRequest
  (:init
   (&key
    ((:display_name __display_name) "")
    ((:racecar_color __racecar_color) "")
    )
   (send-super :init)
   (setq _display_name (string __display_name))
   (setq _racecar_color (string __racecar_color))
   self)
  (:display_name
   (&optional __display_name)
   (if __display_name (setq _display_name __display_name)) _display_name)
  (:racecar_color
   (&optional __racecar_color)
   (if __racecar_color (setq _racecar_color __racecar_color)) _racecar_color)
  (:serialization-length
   ()
   (+
    ;; string _display_name
    4 (length _display_name)
    ;; string _racecar_color
    4 (length _racecar_color)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _display_name
       (write-long (length _display_name) s) (princ _display_name s)
     ;; string _racecar_color
       (write-long (length _racecar_color) s) (princ _racecar_color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _display_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _display_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _racecar_color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _racecar_color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass deepracer_simulation_environment::VirtualEventVideoEditSrvResponse
  :super ros::object
  :slots (_success ))

(defmethod deepracer_simulation_environment::VirtualEventVideoEditSrvResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_simulation_environment::VirtualEventVideoEditSrv
  :super ros::object
  :slots ())

(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrv :md5sum-) "d375256530e7fab3e8486078c126cdb6")
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrv :datatype-) "deepracer_simulation_environment/VirtualEventVideoEditSrv")
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrv :request) deepracer_simulation_environment::VirtualEventVideoEditSrvRequest)
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrv :response) deepracer_simulation_environment::VirtualEventVideoEditSrvResponse)

(defmethod deepracer_simulation_environment::VirtualEventVideoEditSrvRequest
  (:response () (instance deepracer_simulation_environment::VirtualEventVideoEditSrvResponse :init)))

(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrvRequest :md5sum-) "d375256530e7fab3e8486078c126cdb6")
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrvRequest :datatype-) "deepracer_simulation_environment/VirtualEventVideoEditSrvRequest")
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrvRequest :definition-)
      "string display_name
string racecar_color
---
bool success
")

(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrvResponse :md5sum-) "d375256530e7fab3e8486078c126cdb6")
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrvResponse :datatype-) "deepracer_simulation_environment/VirtualEventVideoEditSrvResponse")
(setf (get deepracer_simulation_environment::VirtualEventVideoEditSrvResponse :definition-)
      "string display_name
string racecar_color
---
bool success
")



(provide :deepracer_simulation_environment/VirtualEventVideoEditSrv "d375256530e7fab3e8486078c126cdb6")


