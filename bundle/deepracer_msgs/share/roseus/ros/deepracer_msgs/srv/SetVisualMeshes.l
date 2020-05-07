;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualMeshes)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualMeshes (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALMESHES")
  (make-package "DEEPRACER_MSGS::SETVISUALMESHES"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALMESHESREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALMESHESREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALMESHESRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALMESHESRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass deepracer_msgs::SetVisualMeshesRequest
  :super ros::object
  :slots (_link_names _visual_names _filenames _scales _block ))

(defmethod deepracer_msgs::SetVisualMeshesRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:filenames __filenames) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:scales __scales) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Vector3 :init) r)) r))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   (setq _filenames __filenames)
   (setq _scales __scales)
   (setq _block __block)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:filenames
   (&optional __filenames)
   (if __filenames (setq _filenames __filenames)) _filenames)
  (:scales
   (&rest __scales)
   (if (keywordp (car __scales))
       (send* _scales __scales)
     (progn
       (if __scales (setq _scales (car __scales)))
       _scales)))
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
    ;; string[] _filenames
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _filenames)) 4
    ;; geometry_msgs/Vector3[] _scales
    (apply #'+ (send-all _scales :serialization-length)) 4
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
     ;; string[] _filenames
     (write-long (length _filenames) s)
     (dolist (elem _filenames)
       (write-long (length elem) s) (princ elem s)
       )
     ;; geometry_msgs/Vector3[] _scales
     (write-long (length _scales) s)
     (dolist (elem _scales)
       (send elem :serialize s)
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
   ;; string[] _filenames
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _filenames (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _filenames i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; geometry_msgs/Vector3[] _scales
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scales (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _scales)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualMeshesResponse
  :super ros::object
  :slots (_success _status_message _status _messages ))

(defmethod deepracer_msgs::SetVisualMeshesResponse
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

(defclass deepracer_msgs::SetVisualMeshes
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualMeshes :md5sum-) "a09fb2814d823a8b3a634fc955be2d1a")
(setf (get deepracer_msgs::SetVisualMeshes :datatype-) "deepracer_msgs/SetVisualMeshes")
(setf (get deepracer_msgs::SetVisualMeshes :request) deepracer_msgs::SetVisualMeshesRequest)
(setf (get deepracer_msgs::SetVisualMeshes :response) deepracer_msgs::SetVisualMeshesResponse)

(defmethod deepracer_msgs::SetVisualMeshesRequest
  (:response () (instance deepracer_msgs::SetVisualMeshesResponse :init)))

(setf (get deepracer_msgs::SetVisualMeshesRequest :md5sum-) "a09fb2814d823a8b3a634fc955be2d1a")
(setf (get deepracer_msgs::SetVisualMeshesRequest :datatype-) "deepracer_msgs/SetVisualMeshesRequest")
(setf (get deepracer_msgs::SetVisualMeshesRequest :definition-)
      "string[] link_names
string[] visual_names
string[] filenames
geometry_msgs/Vector3[] scales
bool block

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success
string status_message
int8[] status
string[] messages
")

(setf (get deepracer_msgs::SetVisualMeshesResponse :md5sum-) "a09fb2814d823a8b3a634fc955be2d1a")
(setf (get deepracer_msgs::SetVisualMeshesResponse :datatype-) "deepracer_msgs/SetVisualMeshesResponse")
(setf (get deepracer_msgs::SetVisualMeshesResponse :definition-)
      "string[] link_names
string[] visual_names
string[] filenames
geometry_msgs/Vector3[] scales
bool block

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool success
string status_message
int8[] status
string[] messages
")



(provide :deepracer_msgs/SetVisualMeshes "a09fb2814d823a8b3a634fc955be2d1a")


