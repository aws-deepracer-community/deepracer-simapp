;; Auto-generated. Do not edit!


(when (boundp 'robomaker_simulation_msgs::RemoveTags)
  (if (not (find-package "ROBOMAKER_SIMULATION_MSGS"))
    (make-package "ROBOMAKER_SIMULATION_MSGS"))
  (shadow 'RemoveTags (find-package "ROBOMAKER_SIMULATION_MSGS")))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::REMOVETAGS")
  (make-package "ROBOMAKER_SIMULATION_MSGS::REMOVETAGS"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::REMOVETAGSREQUEST")
  (make-package "ROBOMAKER_SIMULATION_MSGS::REMOVETAGSREQUEST"))
(unless (find-package "ROBOMAKER_SIMULATION_MSGS::REMOVETAGSRESPONSE")
  (make-package "ROBOMAKER_SIMULATION_MSGS::REMOVETAGSRESPONSE"))

(in-package "ROS")





(defclass robomaker_simulation_msgs::RemoveTagsRequest
  :super ros::object
  :slots (_keys ))

(defmethod robomaker_simulation_msgs::RemoveTagsRequest
  (:init
   (&key
    ((:keys __keys) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _keys __keys)
   self)
  (:keys
   (&optional __keys)
   (if __keys (setq _keys __keys)) _keys)
  (:serialization-length
   ()
   (+
    ;; string[] _keys
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _keys)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _keys
     (write-long (length _keys) s)
     (dolist (elem _keys)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _keys
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keys (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _keys i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass robomaker_simulation_msgs::RemoveTagsResponse
  :super ros::object
  :slots (_success _message ))

(defmethod robomaker_simulation_msgs::RemoveTagsResponse
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

(defclass robomaker_simulation_msgs::RemoveTags
  :super ros::object
  :slots ())

(setf (get robomaker_simulation_msgs::RemoveTags :md5sum-) "cae332c01603844d352ed026669010ee")
(setf (get robomaker_simulation_msgs::RemoveTags :datatype-) "robomaker_simulation_msgs/RemoveTags")
(setf (get robomaker_simulation_msgs::RemoveTags :request) robomaker_simulation_msgs::RemoveTagsRequest)
(setf (get robomaker_simulation_msgs::RemoveTags :response) robomaker_simulation_msgs::RemoveTagsResponse)

(defmethod robomaker_simulation_msgs::RemoveTagsRequest
  (:response () (instance robomaker_simulation_msgs::RemoveTagsResponse :init)))

(setf (get robomaker_simulation_msgs::RemoveTagsRequest :md5sum-) "cae332c01603844d352ed026669010ee")
(setf (get robomaker_simulation_msgs::RemoveTagsRequest :datatype-) "robomaker_simulation_msgs/RemoveTagsRequest")
(setf (get robomaker_simulation_msgs::RemoveTagsRequest :definition-)
      "














string[] keys
---

bool success



string message
")

(setf (get robomaker_simulation_msgs::RemoveTagsResponse :md5sum-) "cae332c01603844d352ed026669010ee")
(setf (get robomaker_simulation_msgs::RemoveTagsResponse :datatype-) "robomaker_simulation_msgs/RemoveTagsResponse")
(setf (get robomaker_simulation_msgs::RemoveTagsResponse :definition-)
      "














string[] keys
---

bool success



string message
")



(provide :robomaker_simulation_msgs/RemoveTags "cae332c01603844d352ed026669010ee")


