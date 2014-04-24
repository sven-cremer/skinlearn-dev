; Auto-generated. Do not edit!


(cl:in-package neuroadaptive_msgs-srv)


;//! \htmlinclude setCartPose-request.msg.html

(cl:defclass <setCartPose-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass setCartPose-request (<setCartPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setCartPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setCartPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<setCartPose-request> is deprecated: use neuroadaptive_msgs-srv:setCartPose-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <setCartPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:msg-val is deprecated.  Use neuroadaptive_msgs-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setCartPose-request>) ostream)
  "Serializes a message object of type '<setCartPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setCartPose-request>) istream)
  "Deserializes a message object of type '<setCartPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setCartPose-request>)))
  "Returns string type for a service object of type '<setCartPose-request>"
  "neuroadaptive_msgs/setCartPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setCartPose-request)))
  "Returns string type for a service object of type 'setCartPose-request"
  "neuroadaptive_msgs/setCartPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setCartPose-request>)))
  "Returns md5sum for a message object of type '<setCartPose-request>"
  "674895731e081d8b5c47ecc3c04378d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setCartPose-request)))
  "Returns md5sum for a message object of type 'setCartPose-request"
  "674895731e081d8b5c47ecc3c04378d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setCartPose-request>)))
  "Returns full string definition for message of type '<setCartPose-request>"
  (cl:format cl:nil "~%geometry_msgs/Pose msg~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setCartPose-request)))
  "Returns full string definition for message of type 'setCartPose-request"
  (cl:format cl:nil "~%geometry_msgs/Pose msg~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setCartPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setCartPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setCartPose-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude setCartPose-response.msg.html

(cl:defclass <setCartPose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setCartPose-response (<setCartPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setCartPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setCartPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<setCartPose-response> is deprecated: use neuroadaptive_msgs-srv:setCartPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <setCartPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:success-val is deprecated.  Use neuroadaptive_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setCartPose-response>) ostream)
  "Serializes a message object of type '<setCartPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setCartPose-response>) istream)
  "Deserializes a message object of type '<setCartPose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setCartPose-response>)))
  "Returns string type for a service object of type '<setCartPose-response>"
  "neuroadaptive_msgs/setCartPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setCartPose-response)))
  "Returns string type for a service object of type 'setCartPose-response"
  "neuroadaptive_msgs/setCartPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setCartPose-response>)))
  "Returns md5sum for a message object of type '<setCartPose-response>"
  "674895731e081d8b5c47ecc3c04378d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setCartPose-response)))
  "Returns md5sum for a message object of type 'setCartPose-response"
  "674895731e081d8b5c47ecc3c04378d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setCartPose-response>)))
  "Returns full string definition for message of type '<setCartPose-response>"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setCartPose-response)))
  "Returns full string definition for message of type 'setCartPose-response"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setCartPose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setCartPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setCartPose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setCartPose)))
  'setCartPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setCartPose)))
  'setCartPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setCartPose)))
  "Returns string type for a service object of type '<setCartPose>"
  "neuroadaptive_msgs/setCartPose")