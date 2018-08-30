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
    ((:return_value __return_value) 0)
    )
   (send-super :init)
   (setq _return_value (round __return_value))
   self)
  (:return_value
   (&optional __return_value)
   (if __return_value (setq _return_value __return_value)) _return_value)
  (:serialization-length
   ()
   (+
    ;; uint8 _return_value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _return_value
       (write-byte _return_value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _return_value
     (setq _return_value (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass solar_buggy::Controller
  :super ros::object
  :slots ())

(setf (get solar_buggy::Controller :md5sum-) "770770ca4208a49b403ae37e8aa24498")
(setf (get solar_buggy::Controller :datatype-) "solar_buggy/Controller")
(setf (get solar_buggy::Controller :request) solar_buggy::ControllerRequest)
(setf (get solar_buggy::Controller :response) solar_buggy::ControllerResponse)

(defmethod solar_buggy::ControllerRequest
  (:response () (instance solar_buggy::ControllerResponse :init)))

(setf (get solar_buggy::ControllerRequest :md5sum-) "770770ca4208a49b403ae37e8aa24498")
(setf (get solar_buggy::ControllerRequest :datatype-) "solar_buggy/ControllerRequest")
(setf (get solar_buggy::ControllerRequest :definition-)
      "---
uint8 return_value

")

(setf (get solar_buggy::ControllerResponse :md5sum-) "770770ca4208a49b403ae37e8aa24498")
(setf (get solar_buggy::ControllerResponse :datatype-) "solar_buggy/ControllerResponse")
(setf (get solar_buggy::ControllerResponse :definition-)
      "---
uint8 return_value

")



(provide :solar_buggy/Controller "770770ca4208a49b403ae37e8aa24498")


