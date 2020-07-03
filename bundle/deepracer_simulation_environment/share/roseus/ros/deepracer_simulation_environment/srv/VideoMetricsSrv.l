;; Auto-generated. Do not edit!


(when (boundp 'deepracer_simulation_environment::VideoMetricsSrv)
  (if (not (find-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
    (make-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
  (shadow 'VideoMetricsSrv (find-package "DEEPRACER_SIMULATION_ENVIRONMENT")))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIDEOMETRICSSRV")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIDEOMETRICSSRV"))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIDEOMETRICSSRVREQUEST")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIDEOMETRICSSRVREQUEST"))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIDEOMETRICSSRVRESPONSE")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::VIDEOMETRICSSRVRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass deepracer_simulation_environment::VideoMetricsSrvRequest
  :super ros::object
  :slots ())

(defmethod deepracer_simulation_environment::VideoMetricsSrvRequest
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

(defclass deepracer_simulation_environment::VideoMetricsSrvResponse
  :super ros::object
  :slots (_lap_counter _completion_percentage _reset_counter _throttle _steering _best_lap_time _total_evaluation_time _done _x _y _object_locations ))

(defmethod deepracer_simulation_environment::VideoMetricsSrvResponse
  (:init
   (&key
    ((:lap_counter __lap_counter) 0.0)
    ((:completion_percentage __completion_percentage) 0.0)
    ((:reset_counter __reset_counter) 0)
    ((:throttle __throttle) 0.0)
    ((:steering __steering) 0.0)
    ((:best_lap_time __best_lap_time) 0.0)
    ((:total_evaluation_time __total_evaluation_time) 0.0)
    ((:done __done) nil)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:object_locations __object_locations) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point32 :init) r)) r))
    )
   (send-super :init)
   (setq _lap_counter (float __lap_counter))
   (setq _completion_percentage (float __completion_percentage))
   (setq _reset_counter (round __reset_counter))
   (setq _throttle (float __throttle))
   (setq _steering (float __steering))
   (setq _best_lap_time (float __best_lap_time))
   (setq _total_evaluation_time (float __total_evaluation_time))
   (setq _done __done)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _object_locations __object_locations)
   self)
  (:lap_counter
   (&optional __lap_counter)
   (if __lap_counter (setq _lap_counter __lap_counter)) _lap_counter)
  (:completion_percentage
   (&optional __completion_percentage)
   (if __completion_percentage (setq _completion_percentage __completion_percentage)) _completion_percentage)
  (:reset_counter
   (&optional __reset_counter)
   (if __reset_counter (setq _reset_counter __reset_counter)) _reset_counter)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:best_lap_time
   (&optional __best_lap_time)
   (if __best_lap_time (setq _best_lap_time __best_lap_time)) _best_lap_time)
  (:total_evaluation_time
   (&optional __total_evaluation_time)
   (if __total_evaluation_time (setq _total_evaluation_time __total_evaluation_time)) _total_evaluation_time)
  (:done
   (&optional __done)
   (if __done (setq _done __done)) _done)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:object_locations
   (&rest __object_locations)
   (if (keywordp (car __object_locations))
       (send* _object_locations __object_locations)
     (progn
       (if __object_locations (setq _object_locations (car __object_locations)))
       _object_locations)))
  (:serialization-length
   ()
   (+
    ;; float32 _lap_counter
    4
    ;; float32 _completion_percentage
    4
    ;; int32 _reset_counter
    4
    ;; float32 _throttle
    4
    ;; float32 _steering
    4
    ;; float32 _best_lap_time
    4
    ;; float32 _total_evaluation_time
    4
    ;; bool _done
    1
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; geometry_msgs/Point32[] _object_locations
    (apply #'+ (send-all _object_locations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _lap_counter
       (sys::poke _lap_counter (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _completion_percentage
       (sys::poke _completion_percentage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _reset_counter
       (write-long _reset_counter s)
     ;; float32 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _best_lap_time
       (sys::poke _best_lap_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_evaluation_time
       (sys::poke _total_evaluation_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point32[] _object_locations
     (write-long (length _object_locations) s)
     (dolist (elem _object_locations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _lap_counter
     (setq _lap_counter (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _completion_percentage
     (setq _completion_percentage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _reset_counter
     (setq _reset_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _throttle
     (setq _throttle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering
     (setq _steering (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _best_lap_time
     (setq _best_lap_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_evaluation_time
     (setq _total_evaluation_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point32[] _object_locations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_locations (let (r) (dotimes (i n) (push (instance geometry_msgs::Point32 :init) r)) r))
     (dolist (elem- _object_locations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass deepracer_simulation_environment::VideoMetricsSrv
  :super ros::object
  :slots ())

(setf (get deepracer_simulation_environment::VideoMetricsSrv :md5sum-) "ccf1b22780e2ca6fb64b60963df11d26")
(setf (get deepracer_simulation_environment::VideoMetricsSrv :datatype-) "deepracer_simulation_environment/VideoMetricsSrv")
(setf (get deepracer_simulation_environment::VideoMetricsSrv :request) deepracer_simulation_environment::VideoMetricsSrvRequest)
(setf (get deepracer_simulation_environment::VideoMetricsSrv :response) deepracer_simulation_environment::VideoMetricsSrvResponse)

(defmethod deepracer_simulation_environment::VideoMetricsSrvRequest
  (:response () (instance deepracer_simulation_environment::VideoMetricsSrvResponse :init)))

(setf (get deepracer_simulation_environment::VideoMetricsSrvRequest :md5sum-) "ccf1b22780e2ca6fb64b60963df11d26")
(setf (get deepracer_simulation_environment::VideoMetricsSrvRequest :datatype-) "deepracer_simulation_environment/VideoMetricsSrvRequest")
(setf (get deepracer_simulation_environment::VideoMetricsSrvRequest :definition-)
      "---
float32 lap_counter
float32 completion_percentage
int32 reset_counter
float32 throttle
float32 steering
float32 best_lap_time
float32 total_evaluation_time
bool done
float32 x
float32 y
geometry_msgs/Point32[] object_locations

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
")

(setf (get deepracer_simulation_environment::VideoMetricsSrvResponse :md5sum-) "ccf1b22780e2ca6fb64b60963df11d26")
(setf (get deepracer_simulation_environment::VideoMetricsSrvResponse :datatype-) "deepracer_simulation_environment/VideoMetricsSrvResponse")
(setf (get deepracer_simulation_environment::VideoMetricsSrvResponse :definition-)
      "---
float32 lap_counter
float32 completion_percentage
int32 reset_counter
float32 throttle
float32 steering
float32 best_lap_time
float32 total_evaluation_time
bool done
float32 x
float32 y
geometry_msgs/Point32[] object_locations

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
")



(provide :deepracer_simulation_environment/VideoMetricsSrv "ccf1b22780e2ca6fb64b60963df11d26")


