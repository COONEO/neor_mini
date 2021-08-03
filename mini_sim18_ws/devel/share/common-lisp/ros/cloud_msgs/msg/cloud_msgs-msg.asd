
(cl:in-package :asdf)

(defsystem "cloud_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "cloud_info" :depends-on ("_package_cloud_info"))
    (:file "_package_cloud_info" :depends-on ("_package"))
  ))