;; Auto-generated. Do not edit!


(when (boundp 'robomaker_simulation_msgs::Cancel)
  (if (not (find-package "ROBOMAKER_SIMULATION_MSGS"))
    (make-package "ROBOMAKER_SIMULATION_MSGS"))
  (shadow 'Cancel (find-package "ROBOMAKER_SIMULATION_MSGS")))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::CANCEL")
  (make-package "ROBOMAKER_SIMULATION_MSGS::CANCEL"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::CANCELREQUEST")
  (make-package "ROBOMAKER_SIMULATION_MSGS::CANCELREQUEST"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::CANCELRESPONSE")
  (make-package "ROBOMAKER_SIMULATION_MSGS::CANCELRESPONSE"))

(in-package "ROS")





(defclass robomaker_simulation_msgs::CancelRequest
  :super ros::object
  :slots ())

(defmethod robomaker_simulation_msgs::CancelRequest
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

(defclass robomaker_simulation_msgs::CancelResponse
  :super ros::object
  :slots (_success _message ))

(defmethod robomaker_simulation_msgs::CancelResponse
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

(defclass robomaker_simulation_msgs::Cancel
  :super ros::object
  :slots ())

(setf (get robomaker_simulation_msgs::Cancel :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get robomaker_simulation_msgs::Cancel :datatype-) "robomaker_simulation_msgs/Cancel")
(setf (get robomaker_simulation_msgs::Cancel :request) robomaker_simulation_msgs::CancelRequest)
(setf (get robomaker_simulation_msgs::Cancel :response) robomaker_simulation_msgs::CancelResponse)

(defmethod robomaker_simulation_msgs::CancelRequest
  (:response () (instance robomaker_simulation_msgs::CancelResponse :init)))

(setf (get robomaker_simulation_msgs::CancelRequest :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get robomaker_simulation_msgs::CancelRequest :datatype-) "robomaker_simulation_msgs/CancelRequest")
(setf (get robomaker_simulation_msgs::CancelRequest :definition-)
      "












---

bool success



string message
")

(setf (get robomaker_simulation_msgs::CancelResponse :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get robomaker_simulation_msgs::CancelResponse :datatype-) "robomaker_simulation_msgs/CancelResponse")
(setf (get robomaker_simulation_msgs::CancelResponse :definition-)
      "












---

bool success



string message
")



(provide :robomaker_simulation_msgs/Cancel "937c9679a518e3a18d831e57125ea522")


