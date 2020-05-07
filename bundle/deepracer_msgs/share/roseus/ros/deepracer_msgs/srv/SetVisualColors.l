;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualColors)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualColors (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALCOLORS")
  (make-package "DEEPRACER_MSGS::SETVISUALCOLORS"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALCOLORSREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALCOLORSREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALCOLORSRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALCOLORSRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass deepracer_msgs::SetVisualColorsRequest
  :super ros::object
  :slots (_link_names _visual_names _ambients _diffuses _speculars _emissives _block ))

(defmethod deepracer_msgs::SetVisualColorsRequest
  (:init
   (&key
    ((:link_names __link_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:visual_names __visual_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:ambients __ambients) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:diffuses __diffuses) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:speculars __speculars) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:emissives __emissives) (let (r) (dotimes (i 0) (push (instance std_msgs::ColorRGBA :init) r)) r))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_names __link_names)
   (setq _visual_names __visual_names)
   (setq _ambients __ambients)
   (setq _diffuses __diffuses)
   (setq _speculars __speculars)
   (setq _emissives __emissives)
   (setq _block __block)
   self)
  (:link_names
   (&optional __link_names)
   (if __link_names (setq _link_names __link_names)) _link_names)
  (:visual_names
   (&optional __visual_names)
   (if __visual_names (setq _visual_names __visual_names)) _visual_names)
  (:ambients
   (&rest __ambients)
   (if (keywordp (car __ambients))
       (send* _ambients __ambients)
     (progn
       (if __ambients (setq _ambients (car __ambients)))
       _ambients)))
  (:diffuses
   (&rest __diffuses)
   (if (keywordp (car __diffuses))
       (send* _diffuses __diffuses)
     (progn
       (if __diffuses (setq _diffuses (car __diffuses)))
       _diffuses)))
  (:speculars
   (&rest __speculars)
   (if (keywordp (car __speculars))
       (send* _speculars __speculars)
     (progn
       (if __speculars (setq _speculars (car __speculars)))
       _speculars)))
  (:emissives
   (&rest __emissives)
   (if (keywordp (car __emissives))
       (send* _emissives __emissives)
     (progn
       (if __emissives (setq _emissives (car __emissives)))
       _emissives)))
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
    ;; std_msgs/ColorRGBA[] _ambients
    (apply #'+ (send-all _ambients :serialization-length)) 4
    ;; std_msgs/ColorRGBA[] _diffuses
    (apply #'+ (send-all _diffuses :serialization-length)) 4
    ;; std_msgs/ColorRGBA[] _speculars
    (apply #'+ (send-all _speculars :serialization-length)) 4
    ;; std_msgs/ColorRGBA[] _emissives
    (apply #'+ (send-all _emissives :serialization-length)) 4
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
     ;; std_msgs/ColorRGBA[] _ambients
     (write-long (length _ambients) s)
     (dolist (elem _ambients)
       (send elem :serialize s)
       )
     ;; std_msgs/ColorRGBA[] _diffuses
     (write-long (length _diffuses) s)
     (dolist (elem _diffuses)
       (send elem :serialize s)
       )
     ;; std_msgs/ColorRGBA[] _speculars
     (write-long (length _speculars) s)
     (dolist (elem _speculars)
       (send elem :serialize s)
       )
     ;; std_msgs/ColorRGBA[] _emissives
     (write-long (length _emissives) s)
     (dolist (elem _emissives)
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
   ;; std_msgs/ColorRGBA[] _ambients
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ambients (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _ambients)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/ColorRGBA[] _diffuses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _diffuses (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _diffuses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/ColorRGBA[] _speculars
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speculars (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _speculars)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/ColorRGBA[] _emissives
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _emissives (let (r) (dotimes (i n) (push (instance std_msgs::ColorRGBA :init) r)) r))
     (dolist (elem- _emissives)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualColorsResponse
  :super ros::object
  :slots (_success _status_message _status _messages ))

(defmethod deepracer_msgs::SetVisualColorsResponse
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

(defclass deepracer_msgs::SetVisualColors
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualColors :md5sum-) "2b593b9606746213e8e7b797a0ade086")
(setf (get deepracer_msgs::SetVisualColors :datatype-) "deepracer_msgs/SetVisualColors")
(setf (get deepracer_msgs::SetVisualColors :request) deepracer_msgs::SetVisualColorsRequest)
(setf (get deepracer_msgs::SetVisualColors :response) deepracer_msgs::SetVisualColorsResponse)

(defmethod deepracer_msgs::SetVisualColorsRequest
  (:response () (instance deepracer_msgs::SetVisualColorsResponse :init)))

(setf (get deepracer_msgs::SetVisualColorsRequest :md5sum-) "2b593b9606746213e8e7b797a0ade086")
(setf (get deepracer_msgs::SetVisualColorsRequest :datatype-) "deepracer_msgs/SetVisualColorsRequest")
(setf (get deepracer_msgs::SetVisualColorsRequest :definition-)
      "string[] link_names
string[] visual_names
std_msgs/ColorRGBA[] ambients
std_msgs/ColorRGBA[] diffuses
std_msgs/ColorRGBA[] speculars
std_msgs/ColorRGBA[] emissives
bool block

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
---
bool success
string status_message
int8[] status
string[] messages
")

(setf (get deepracer_msgs::SetVisualColorsResponse :md5sum-) "2b593b9606746213e8e7b797a0ade086")
(setf (get deepracer_msgs::SetVisualColorsResponse :datatype-) "deepracer_msgs/SetVisualColorsResponse")
(setf (get deepracer_msgs::SetVisualColorsResponse :definition-)
      "string[] link_names
string[] visual_names
std_msgs/ColorRGBA[] ambients
std_msgs/ColorRGBA[] diffuses
std_msgs/ColorRGBA[] speculars
std_msgs/ColorRGBA[] emissives
bool block

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
---
bool success
string status_message
int8[] status
string[] messages
")



(provide :deepracer_msgs/SetVisualColors "2b593b9606746213e8e7b797a0ade086")


