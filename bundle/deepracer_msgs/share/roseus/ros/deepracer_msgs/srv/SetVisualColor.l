;; Auto-generated. Do not edit!


(when (boundp 'deepracer_msgs::SetVisualColor)
  (if (not (find-package "DEEPRACER_MSGS"))
    (make-package "DEEPRACER_MSGS"))
  (shadow 'SetVisualColor (find-package "DEEPRACER_MSGS")))
(unless (find-package "DEEPRACER_MSGS::SETVISUALCOLOR")
  (make-package "DEEPRACER_MSGS::SETVISUALCOLOR"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALCOLORREQUEST")
  (make-package "DEEPRACER_MSGS::SETVISUALCOLORREQUEST"))
(unless (find-package "DEEPRACER_MSGS::SETVISUALCOLORRESPONSE")
  (make-package "DEEPRACER_MSGS::SETVISUALCOLORRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass deepracer_msgs::SetVisualColorRequest
  :super ros::object
  :slots (_link_name _visual_name _ambient _diffuse _specular _emissive _block ))

(defmethod deepracer_msgs::SetVisualColorRequest
  (:init
   (&key
    ((:link_name __link_name) "")
    ((:visual_name __visual_name) "")
    ((:ambient __ambient) (instance std_msgs::ColorRGBA :init))
    ((:diffuse __diffuse) (instance std_msgs::ColorRGBA :init))
    ((:specular __specular) (instance std_msgs::ColorRGBA :init))
    ((:emissive __emissive) (instance std_msgs::ColorRGBA :init))
    ((:block __block) nil)
    )
   (send-super :init)
   (setq _link_name (string __link_name))
   (setq _visual_name (string __visual_name))
   (setq _ambient __ambient)
   (setq _diffuse __diffuse)
   (setq _specular __specular)
   (setq _emissive __emissive)
   (setq _block __block)
   self)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:visual_name
   (&optional __visual_name)
   (if __visual_name (setq _visual_name __visual_name)) _visual_name)
  (:ambient
   (&rest __ambient)
   (if (keywordp (car __ambient))
       (send* _ambient __ambient)
     (progn
       (if __ambient (setq _ambient (car __ambient)))
       _ambient)))
  (:diffuse
   (&rest __diffuse)
   (if (keywordp (car __diffuse))
       (send* _diffuse __diffuse)
     (progn
       (if __diffuse (setq _diffuse (car __diffuse)))
       _diffuse)))
  (:specular
   (&rest __specular)
   (if (keywordp (car __specular))
       (send* _specular __specular)
     (progn
       (if __specular (setq _specular (car __specular)))
       _specular)))
  (:emissive
   (&rest __emissive)
   (if (keywordp (car __emissive))
       (send* _emissive __emissive)
     (progn
       (if __emissive (setq _emissive (car __emissive)))
       _emissive)))
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
    ;; std_msgs/ColorRGBA _ambient
    (send _ambient :serialization-length)
    ;; std_msgs/ColorRGBA _diffuse
    (send _diffuse :serialization-length)
    ;; std_msgs/ColorRGBA _specular
    (send _specular :serialization-length)
    ;; std_msgs/ColorRGBA _emissive
    (send _emissive :serialization-length)
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
     ;; std_msgs/ColorRGBA _ambient
       (send _ambient :serialize s)
     ;; std_msgs/ColorRGBA _diffuse
       (send _diffuse :serialize s)
     ;; std_msgs/ColorRGBA _specular
       (send _specular :serialize s)
     ;; std_msgs/ColorRGBA _emissive
       (send _emissive :serialize s)
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
   ;; std_msgs/ColorRGBA _ambient
     (send _ambient :deserialize buf ptr-) (incf ptr- (send _ambient :serialization-length))
   ;; std_msgs/ColorRGBA _diffuse
     (send _diffuse :deserialize buf ptr-) (incf ptr- (send _diffuse :serialization-length))
   ;; std_msgs/ColorRGBA _specular
     (send _specular :deserialize buf ptr-) (incf ptr- (send _specular :serialization-length))
   ;; std_msgs/ColorRGBA _emissive
     (send _emissive :deserialize buf ptr-) (incf ptr- (send _emissive :serialization-length))
   ;; bool _block
     (setq _block (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass deepracer_msgs::SetVisualColorResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod deepracer_msgs::SetVisualColorResponse
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

(defclass deepracer_msgs::SetVisualColor
  :super ros::object
  :slots ())

(setf (get deepracer_msgs::SetVisualColor :md5sum-) "9c987659e93e8e993b90a6ea6fab5b74")
(setf (get deepracer_msgs::SetVisualColor :datatype-) "deepracer_msgs/SetVisualColor")
(setf (get deepracer_msgs::SetVisualColor :request) deepracer_msgs::SetVisualColorRequest)
(setf (get deepracer_msgs::SetVisualColor :response) deepracer_msgs::SetVisualColorResponse)

(defmethod deepracer_msgs::SetVisualColorRequest
  (:response () (instance deepracer_msgs::SetVisualColorResponse :init)))

(setf (get deepracer_msgs::SetVisualColorRequest :md5sum-) "9c987659e93e8e993b90a6ea6fab5b74")
(setf (get deepracer_msgs::SetVisualColorRequest :datatype-) "deepracer_msgs/SetVisualColorRequest")
(setf (get deepracer_msgs::SetVisualColorRequest :definition-)
      "string link_name
string visual_name
std_msgs/ColorRGBA ambient
std_msgs/ColorRGBA diffuse
std_msgs/ColorRGBA specular
std_msgs/ColorRGBA emissive
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
")

(setf (get deepracer_msgs::SetVisualColorResponse :md5sum-) "9c987659e93e8e993b90a6ea6fab5b74")
(setf (get deepracer_msgs::SetVisualColorResponse :datatype-) "deepracer_msgs/SetVisualColorResponse")
(setf (get deepracer_msgs::SetVisualColorResponse :definition-)
      "string link_name
string visual_name
std_msgs/ColorRGBA ambient
std_msgs/ColorRGBA diffuse
std_msgs/ColorRGBA specular
std_msgs/ColorRGBA emissive
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
")



(provide :deepracer_msgs/SetVisualColor "9c987659e93e8e993b90a6ea6fab5b74")


