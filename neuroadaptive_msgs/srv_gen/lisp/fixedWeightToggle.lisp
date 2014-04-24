; Auto-generated. Do not edit!


(cl:in-package neuroadaptive_msgs-srv)


;//! \htmlinclude fixedWeightToggle-request.msg.html

(cl:defclass <fixedWeightToggle-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass fixedWeightToggle-request (<fixedWeightToggle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <fixedWeightToggle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'fixedWeightToggle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<fixedWeightToggle-request> is deprecated: use neuroadaptive_msgs-srv:fixedWeightToggle-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <fixedWeightToggle-request>) ostream)
  "Serializes a message object of type '<fixedWeightToggle-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <fixedWeightToggle-request>) istream)
  "Deserializes a message object of type '<fixedWeightToggle-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<fixedWeightToggle-request>)))
  "Returns string type for a service object of type '<fixedWeightToggle-request>"
  "neuroadaptive_msgs/fixedWeightToggleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fixedWeightToggle-request)))
  "Returns string type for a service object of type 'fixedWeightToggle-request"
  "neuroadaptive_msgs/fixedWeightToggleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<fixedWeightToggle-request>)))
  "Returns md5sum for a message object of type '<fixedWeightToggle-request>"
  "02dff059c4f4a1b745fefa8ea7956d11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'fixedWeightToggle-request)))
  "Returns md5sum for a message object of type 'fixedWeightToggle-request"
  "02dff059c4f4a1b745fefa8ea7956d11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<fixedWeightToggle-request>)))
  "Returns full string definition for message of type '<fixedWeightToggle-request>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'fixedWeightToggle-request)))
  "Returns full string definition for message of type 'fixedWeightToggle-request"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <fixedWeightToggle-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <fixedWeightToggle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'fixedWeightToggle-request
))
;//! \htmlinclude fixedWeightToggle-response.msg.html

(cl:defclass <fixedWeightToggle-response> (roslisp-msg-protocol:ros-message)
  ((useFixedWeights
    :reader useFixedWeights
    :initarg :useFixedWeights
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass fixedWeightToggle-response (<fixedWeightToggle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <fixedWeightToggle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'fixedWeightToggle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<fixedWeightToggle-response> is deprecated: use neuroadaptive_msgs-srv:fixedWeightToggle-response instead.")))

(cl:ensure-generic-function 'useFixedWeights-val :lambda-list '(m))
(cl:defmethod useFixedWeights-val ((m <fixedWeightToggle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:useFixedWeights-val is deprecated.  Use neuroadaptive_msgs-srv:useFixedWeights instead.")
  (useFixedWeights m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <fixedWeightToggle-response>) ostream)
  "Serializes a message object of type '<fixedWeightToggle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'useFixedWeights) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <fixedWeightToggle-response>) istream)
  "Deserializes a message object of type '<fixedWeightToggle-response>"
    (cl:setf (cl:slot-value msg 'useFixedWeights) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<fixedWeightToggle-response>)))
  "Returns string type for a service object of type '<fixedWeightToggle-response>"
  "neuroadaptive_msgs/fixedWeightToggleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fixedWeightToggle-response)))
  "Returns string type for a service object of type 'fixedWeightToggle-response"
  "neuroadaptive_msgs/fixedWeightToggleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<fixedWeightToggle-response>)))
  "Returns md5sum for a message object of type '<fixedWeightToggle-response>"
  "02dff059c4f4a1b745fefa8ea7956d11")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'fixedWeightToggle-response)))
  "Returns md5sum for a message object of type 'fixedWeightToggle-response"
  "02dff059c4f4a1b745fefa8ea7956d11")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<fixedWeightToggle-response>)))
  "Returns full string definition for message of type '<fixedWeightToggle-response>"
  (cl:format cl:nil "~%bool useFixedWeights~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'fixedWeightToggle-response)))
  "Returns full string definition for message of type 'fixedWeightToggle-response"
  (cl:format cl:nil "~%bool useFixedWeights~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <fixedWeightToggle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <fixedWeightToggle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'fixedWeightToggle-response
    (cl:cons ':useFixedWeights (useFixedWeights msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'fixedWeightToggle)))
  'fixedWeightToggle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'fixedWeightToggle)))
  'fixedWeightToggle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'fixedWeightToggle)))
  "Returns string type for a service object of type '<fixedWeightToggle>"
  "neuroadaptive_msgs/fixedWeightToggle")