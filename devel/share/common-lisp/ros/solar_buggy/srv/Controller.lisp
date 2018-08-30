; Auto-generated. Do not edit!


(cl:in-package solar_buggy-srv)


;//! \htmlinclude Controller-request.msg.html

(cl:defclass <Controller-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Controller-request (<Controller-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Controller-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Controller-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name solar_buggy-srv:<Controller-request> is deprecated: use solar_buggy-srv:Controller-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Controller-request>) ostream)
  "Serializes a message object of type '<Controller-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Controller-request>) istream)
  "Deserializes a message object of type '<Controller-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Controller-request>)))
  "Returns string type for a service object of type '<Controller-request>"
  "solar_buggy/ControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Controller-request)))
  "Returns string type for a service object of type 'Controller-request"
  "solar_buggy/ControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Controller-request>)))
  "Returns md5sum for a message object of type '<Controller-request>"
  "770770ca4208a49b403ae37e8aa24498")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Controller-request)))
  "Returns md5sum for a message object of type 'Controller-request"
  "770770ca4208a49b403ae37e8aa24498")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Controller-request>)))
  "Returns full string definition for message of type '<Controller-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Controller-request)))
  "Returns full string definition for message of type 'Controller-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Controller-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Controller-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Controller-request
))
;//! \htmlinclude Controller-response.msg.html

(cl:defclass <Controller-response> (roslisp-msg-protocol:ros-message)
  ((return_value
    :reader return_value
    :initarg :return_value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Controller-response (<Controller-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Controller-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Controller-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name solar_buggy-srv:<Controller-response> is deprecated: use solar_buggy-srv:Controller-response instead.")))

(cl:ensure-generic-function 'return_value-val :lambda-list '(m))
(cl:defmethod return_value-val ((m <Controller-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader solar_buggy-srv:return_value-val is deprecated.  Use solar_buggy-srv:return_value instead.")
  (return_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Controller-response>) ostream)
  "Serializes a message object of type '<Controller-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'return_value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Controller-response>) istream)
  "Deserializes a message object of type '<Controller-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'return_value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Controller-response>)))
  "Returns string type for a service object of type '<Controller-response>"
  "solar_buggy/ControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Controller-response)))
  "Returns string type for a service object of type 'Controller-response"
  "solar_buggy/ControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Controller-response>)))
  "Returns md5sum for a message object of type '<Controller-response>"
  "770770ca4208a49b403ae37e8aa24498")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Controller-response)))
  "Returns md5sum for a message object of type 'Controller-response"
  "770770ca4208a49b403ae37e8aa24498")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Controller-response>)))
  "Returns full string definition for message of type '<Controller-response>"
  (cl:format cl:nil "uint8 return_value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Controller-response)))
  "Returns full string definition for message of type 'Controller-response"
  (cl:format cl:nil "uint8 return_value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Controller-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Controller-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Controller-response
    (cl:cons ':return_value (return_value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Controller)))
  'Controller-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Controller)))
  'Controller-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Controller)))
  "Returns string type for a service object of type '<Controller>"
  "solar_buggy/Controller")