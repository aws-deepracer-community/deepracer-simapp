;; Auto-generated. Do not edit!


(when (boundp 'deepracer_simulation_environment::TopCamDataSrv)
  (if (not (find-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
    (make-package "DEEPRACER_SIMULATION_ENVIRONMENT"))
  (shadow 'TopCamDataSrv (find-package "DEEPRACER_SIMULATION_ENVIRONMENT")))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::TOPCAMDATASRV")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::TOPCAMDATASRV"))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::TOPCAMDATASRVREQUEST")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::TOPCAMDATASRVREQUEST"))
(unless (find-package "DEEPRACER_SIMULATION_ENVIRONMENT::TOPCAMDATASRVRESPONSE")
  (make-package "DEEPRACER_SIMULATION_ENVIRONMENT::TOPCAMDATASRVRESPONSE"))

(in-package "ROS")





(defclass deepracer_simulation_environment::TopCamDataSrvRequest
  :super ros::object
  :slots ())

(defmethod deepracer_simulation_environment::TopCamDataSrvRequest
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

(defclass deepracer_simulation_environment::TopCamDataSrvResponse
  :super ros::object
  :slots (_horizontal_fov _padding_pct _image_width _image_height ))

(defmethod deepracer_simulation_environment::TopCamDataSrvResponse
  (:init
   (&key
    ((:horizontal_fov __horizontal_fov) 0.0)
    ((:padding_pct __padding_pct) 0.0)
    ((:image_width __image_width) 0.0)
    ((:image_height __image_height) 0.0)
    )
   (send-super :init)
   (setq _horizontal_fov (float __horizontal_fov))
   (setq _padding_pct (float __padding_pct))
   (setq _image_width (float __image_width))
   (setq _image_height (float __image_height))
   self)
  (:horizontal_fov
   (&optional __horizontal_fov)
   (if __horizontal_fov (setq _horizontal_fov __horizontal_fov)) _horizontal_fov)
  (:padding_pct
   (&optional __padding_pct)
   (if __padding_pct (setq _padding_pct __padding_pct)) _padding_pct)
  (:image_width
   (&optional __image_width)
   (if __image_width (setq _image_width __image_width)) _image_width)
  (:image_height
   (&optional __image_height)
   (if __image_height (setq _image_height __image_height)) _image_height)
  (:serialization-length
   ()
   (+
    ;; float32 _horizontal_fov
    4
    ;; float32 _padding_pct
    4
    ;; float32 _image_width
    4
    ;; float32 _image_height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _horizontal_fov
       (sys::poke _horizontal_fov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _padding_pct
       (sys::poke _padding_pct (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _image_width
       (sys::poke _image_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _image_height
       (sys::poke _image_height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _horizontal_fov
     (setq _horizontal_fov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _padding_pct
     (setq _padding_pct (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _image_width
     (setq _image_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _image_height
     (setq _image_height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass deepracer_simulation_environment::TopCamDataSrv
  :super ros::object
  :slots ())

(setf (get deepracer_simulation_environment::TopCamDataSrv :md5sum-) "eed9195ebff712c1941314f098f70944")
(setf (get deepracer_simulation_environment::TopCamDataSrv :datatype-) "deepracer_simulation_environment/TopCamDataSrv")
(setf (get deepracer_simulation_environment::TopCamDataSrv :request) deepracer_simulation_environment::TopCamDataSrvRequest)
(setf (get deepracer_simulation_environment::TopCamDataSrv :response) deepracer_simulation_environment::TopCamDataSrvResponse)

(defmethod deepracer_simulation_environment::TopCamDataSrvRequest
  (:response () (instance deepracer_simulation_environment::TopCamDataSrvResponse :init)))

(setf (get deepracer_simulation_environment::TopCamDataSrvRequest :md5sum-) "eed9195ebff712c1941314f098f70944")
(setf (get deepracer_simulation_environment::TopCamDataSrvRequest :datatype-) "deepracer_simulation_environment/TopCamDataSrvRequest")
(setf (get deepracer_simulation_environment::TopCamDataSrvRequest :definition-)
      "---
float32 horizontal_fov
float32 padding_pct
float32 image_width
float32 image_height
")

(setf (get deepracer_simulation_environment::TopCamDataSrvResponse :md5sum-) "eed9195ebff712c1941314f098f70944")
(setf (get deepracer_simulation_environment::TopCamDataSrvResponse :datatype-) "deepracer_simulation_environment/TopCamDataSrvResponse")
(setf (get deepracer_simulation_environment::TopCamDataSrvResponse :definition-)
      "---
float32 horizontal_fov
float32 padding_pct
float32 image_width
float32 image_height
")



(provide :deepracer_simulation_environment/TopCamDataSrv "eed9195ebff712c1941314f098f70944")


