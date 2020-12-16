
(cl:in-package :asdf)

(defsystem "robomaker_simulation_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :robomaker_simulation_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTags" :depends-on ("_package_AddTags"))
    (:file "_package_AddTags" :depends-on ("_package"))
    (:file "Cancel" :depends-on ("_package_Cancel"))
    (:file "_package_Cancel" :depends-on ("_package"))
    (:file "ListTags" :depends-on ("_package_ListTags"))
    (:file "_package_ListTags" :depends-on ("_package"))
    (:file "RemoveTags" :depends-on ("_package_RemoveTags"))
    (:file "_package_RemoveTags" :depends-on ("_package"))
  ))