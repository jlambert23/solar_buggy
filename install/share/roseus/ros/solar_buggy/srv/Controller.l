;; Auto-generated. Do not edit!


(when (boundp 'solar_buggy::Controller)
  (if (not (find-package "SOLAR_BUGGY"))
    (make-package "SOLAR_BUGGY"))
  (shadow 'Controller (find-package "SOLAR_BUGGY")))
(unless (find-package "SOLAR_BUGGY::CONTROLLER")
  (make-package "SOLAR_BUGGY::CONTROLLER"))
(unless (find-package "SOLAR_BUGGY::CONTROLLERREQUEST")
  (make-package "SOLAR_BUGGY::CONTROLLERREQUEST"))
(unless (find-package "SOLAR_BUGGY::CONTROLLERRESPONSE")
  (make-package "SOLAR_BUGGY::CONTROLLERRESPONSE"))

(in-package "ROS")





(defclass solar_buggy::ControllerRequest
  :super ros::object
  :slots ())

(defmethod solar_buggy::ControllerRequest
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

(defclass solar_buggy::ControllerResponse
  :super ros::object
  :slots (_return_value ))

(defmethod solar_buggy::ControllerResponse
  (:init
   (&key
    ((:return_value __return_value) "")
    )
   (send-super :init)
   (setq _return_value (string __return_value))
   self)
  (:return_value
   (&optional __return_value)
   (if __return_value (setq _return_value __return_value)) _return_value)
  (:serialization-length
   ()
   (+
    ;; string _return_value
    4 (length _return_value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _return_value
       (write-long (length _return_value) s) (princ _return_value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _return_value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _return_value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass solar_buggy::Controller
  :super ros::object
  :slots ())

(setf (get solar_buggy::Controller :md5sum-) "b68c06a61da57812b519df2ba93f87ff")
(setf (get solar_buggy::Controller :datatype-) "solar_buggy/Controller")
(setf (get solar_buggy::Controller :request) solar_buggy::ControllerRequest)
(setf (get solar_buggy::Controller :response) solar_buggy::ControllerResponse)

(defmethod solar_buggy::ControllerRequest
  (:response () (instance solar_buggy::ControllerResponse :init)))

(setf (get solar_buggy::ControllerRequest :md5sum-) "b68c06a61da57812b519df2ba93f87ff")
(setf (get solar_buggy::ControllerRequest :datatype-) "solar_buggy/ControllerRequest")
(setf (get solar_buggy::ControllerRequest :definition-)
      "---
string return_value

")

(setf (get solar_buggy::ControllerResponse :md5sum-) "b68c06a61da57812b519df2ba93f87ff")
(setf (get solar_buggy::ControllerResponse :datatype-) "solar_buggy/ControllerResponse")
(setf (get solar_buggy::ControllerResponse :definition-)
      "---
string return_value

")



(provide :solar_buggy/Controller "b68c06a61da57812b519df2ba93f87ff")


