
(cl:in-package :asdf)

(defsystem "deepracer_simulation_environment-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "TopCamDataSrv" :depends-on ("_package_TopCamDataSrv"))
    (:file "_package_TopCamDataSrv" :depends-on ("_package"))
    (:file "VideoMetricsSrv" :depends-on ("_package_VideoMetricsSrv"))
    (:file "_package_VideoMetricsSrv" :depends-on ("_package"))
  ))