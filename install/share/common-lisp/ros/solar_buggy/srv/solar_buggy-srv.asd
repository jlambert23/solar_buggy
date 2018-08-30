
(cl:in-package :asdf)

(defsystem "solar_buggy-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Controller" :depends-on ("_package_Controller"))
    (:file "_package_Controller" :depends-on ("_package"))
  ))