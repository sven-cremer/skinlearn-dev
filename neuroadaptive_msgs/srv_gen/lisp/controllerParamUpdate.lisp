; Auto-generated. Do not edit!


(cl:in-package neuroadaptive_msgs-srv)


;//! \htmlinclude controllerParamUpdate-request.msg.html

(cl:defclass <controllerParamUpdate-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type neuroadaptive_msgs-msg:controllerParam
    :initform (cl:make-instance 'neuroadaptive_msgs-msg:controllerParam)))
)

(cl:defclass controllerParamUpdate-request (<controllerParamUpdate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controllerParamUpdate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controllerParamUpdate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<controllerParamUpdate-request> is deprecated: use neuroadaptive_msgs-srv:controllerParamUpdate-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <controllerParamUpdate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:msg-val is deprecated.  Use neuroadaptive_msgs-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controllerParamUpdate-request>) ostream)
  "Serializes a message object of type '<controllerParamUpdate-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controllerParamUpdate-request>) istream)
  "Deserializes a message object of type '<controllerParamUpdate-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controllerParamUpdate-request>)))
  "Returns string type for a service object of type '<controllerParamUpdate-request>"
  "neuroadaptive_msgs/controllerParamUpdateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controllerParamUpdate-request)))
  "Returns string type for a service object of type 'controllerParamUpdate-request"
  "neuroadaptive_msgs/controllerParamUpdateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controllerParamUpdate-request>)))
  "Returns md5sum for a message object of type '<controllerParamUpdate-request>"
  "c95d6edeed3ff05d22a3f8b98eefd5bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controllerParamUpdate-request)))
  "Returns md5sum for a message object of type 'controllerParamUpdate-request"
  "c95d6edeed3ff05d22a3f8b98eefd5bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controllerParamUpdate-request>)))
  "Returns full string definition for message of type '<controllerParamUpdate-request>"
  (cl:format cl:nil "~%controllerParam msg~%~%================================================================================~%MSG: neuroadaptive_msgs/controllerParam~%Header header~%~%# NN Params~%float64 kappa~%float64 Kv~%float64 lambda~%float64 Kz~%float64 Zb~%float64 F~%float64 G~%int64 inParams~%int64 outParams~%int64 hiddenNodes~%int64 errorParams~%float64 feedForwardForce~%float64 nn_ON~%~%# Cart params~%float64 cartPos_Kp_x~%float64 cartPos_Kp_y~%float64 cartPos_Kp_z~%float64 cartPos_Kd_x~%float64 cartPos_Kd_y~%float64 cartPos_Kd_z~%~%float64 cartRot_Kp_x~%float64 cartRot_Kp_y~%float64 cartRot_Kp_z~%float64 cartRot_Kd_x~%float64 cartRot_Kd_y~%float64 cartRot_Kd_z~%~%bool useCurrentCartPose~%bool useNullspacePose~%~%float64 cartIniX    ~%float64 cartIniY    ~%float64 cartIniZ~%float64 cartIniRoll ~%float64 cartIniPitch~%float64 cartIniYaw  ~%~%float64 cartDesX    ~%float64 cartDesY    ~%float64 cartDesZ    ~%float64 cartDesRoll ~%float64 cartDesPitch~%float64 cartDesYaw  ~%~%# Ref Model Params~%float64 m~%float64 d~%float64 k~%~%float64 task_mA~%float64 task_mB~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controllerParamUpdate-request)))
  "Returns full string definition for message of type 'controllerParamUpdate-request"
  (cl:format cl:nil "~%controllerParam msg~%~%================================================================================~%MSG: neuroadaptive_msgs/controllerParam~%Header header~%~%# NN Params~%float64 kappa~%float64 Kv~%float64 lambda~%float64 Kz~%float64 Zb~%float64 F~%float64 G~%int64 inParams~%int64 outParams~%int64 hiddenNodes~%int64 errorParams~%float64 feedForwardForce~%float64 nn_ON~%~%# Cart params~%float64 cartPos_Kp_x~%float64 cartPos_Kp_y~%float64 cartPos_Kp_z~%float64 cartPos_Kd_x~%float64 cartPos_Kd_y~%float64 cartPos_Kd_z~%~%float64 cartRot_Kp_x~%float64 cartRot_Kp_y~%float64 cartRot_Kp_z~%float64 cartRot_Kd_x~%float64 cartRot_Kd_y~%float64 cartRot_Kd_z~%~%bool useCurrentCartPose~%bool useNullspacePose~%~%float64 cartIniX    ~%float64 cartIniY    ~%float64 cartIniZ~%float64 cartIniRoll ~%float64 cartIniPitch~%float64 cartIniYaw  ~%~%float64 cartDesX    ~%float64 cartDesY    ~%float64 cartDesZ    ~%float64 cartDesRoll ~%float64 cartDesPitch~%float64 cartDesYaw  ~%~%# Ref Model Params~%float64 m~%float64 d~%float64 k~%~%float64 task_mA~%float64 task_mB~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controllerParamUpdate-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controllerParamUpdate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'controllerParamUpdate-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude controllerParamUpdate-response.msg.html

(cl:defclass <controllerParamUpdate-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass controllerParamUpdate-response (<controllerParamUpdate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controllerParamUpdate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controllerParamUpdate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<controllerParamUpdate-response> is deprecated: use neuroadaptive_msgs-srv:controllerParamUpdate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <controllerParamUpdate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:success-val is deprecated.  Use neuroadaptive_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controllerParamUpdate-response>) ostream)
  "Serializes a message object of type '<controllerParamUpdate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controllerParamUpdate-response>) istream)
  "Deserializes a message object of type '<controllerParamUpdate-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controllerParamUpdate-response>)))
  "Returns string type for a service object of type '<controllerParamUpdate-response>"
  "neuroadaptive_msgs/controllerParamUpdateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controllerParamUpdate-response)))
  "Returns string type for a service object of type 'controllerParamUpdate-response"
  "neuroadaptive_msgs/controllerParamUpdateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controllerParamUpdate-response>)))
  "Returns md5sum for a message object of type '<controllerParamUpdate-response>"
  "c95d6edeed3ff05d22a3f8b98eefd5bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controllerParamUpdate-response)))
  "Returns md5sum for a message object of type 'controllerParamUpdate-response"
  "c95d6edeed3ff05d22a3f8b98eefd5bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controllerParamUpdate-response>)))
  "Returns full string definition for message of type '<controllerParamUpdate-response>"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controllerParamUpdate-response)))
  "Returns full string definition for message of type 'controllerParamUpdate-response"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controllerParamUpdate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controllerParamUpdate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'controllerParamUpdate-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'controllerParamUpdate)))
  'controllerParamUpdate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'controllerParamUpdate)))
  'controllerParamUpdate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controllerParamUpdate)))
  "Returns string type for a service object of type '<controllerParamUpdate>"
  "neuroadaptive_msgs/controllerParamUpdate")