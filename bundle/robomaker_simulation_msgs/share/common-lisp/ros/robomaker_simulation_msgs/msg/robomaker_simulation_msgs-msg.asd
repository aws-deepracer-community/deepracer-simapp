
(cl:in-package :asdf)

(defsystem "robomaker_simulation_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Tag" :depends-on ("_package_Tag"))
    (:file "_package_Tag" :depends-on ("_package"))
  ))