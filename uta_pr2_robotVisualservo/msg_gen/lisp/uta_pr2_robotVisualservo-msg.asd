
(cl:in-package :asdf)

(defsystem "uta_pr2_robotVisualservo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "imgFeature" :depends-on ("_package_imgFeature"))
    (:file "_package_imgFeature" :depends-on ("_package"))
  ))