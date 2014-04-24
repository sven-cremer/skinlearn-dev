; Auto-generated. Do not edit!


(cl:in-package neuroadaptive_msgs-srv)


;//! \htmlinclude saveControllerData-request.msg.html

(cl:defclass <saveControllerData-request> (roslisp-msg-protocol:ros-message)
  ((fileName
    :reader fileName
    :initarg :fileName
    :type cl:string
    :initform ""))
)

(cl:defclass saveControllerData-request (<saveControllerData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveControllerData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveControllerData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<saveControllerData-request> is deprecated: use neuroadaptive_msgs-srv:saveControllerData-request instead.")))

(cl:ensure-generic-function 'fileName-val :lambda-list '(m))
(cl:defmethod fileName-val ((m <saveControllerData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:fileName-val is deprecated.  Use neuroadaptive_msgs-srv:fileName instead.")
  (fileName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveControllerData-request>) ostream)
  "Serializes a message object of type '<saveControllerData-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fileName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fileName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveControllerData-request>) istream)
  "Deserializes a message object of type '<saveControllerData-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fileName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fileName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveControllerData-request>)))
  "Returns string type for a service object of type '<saveControllerData-request>"
  "neuroadaptive_msgs/saveControllerDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveControllerData-request)))
  "Returns string type for a service object of type 'saveControllerData-request"
  "neuroadaptive_msgs/saveControllerDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveControllerData-request>)))
  "Returns md5sum for a message object of type '<saveControllerData-request>"
  "71f144908c8613a70a312851a43a7309")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveControllerData-request)))
  "Returns md5sum for a message object of type 'saveControllerData-request"
  "71f144908c8613a70a312851a43a7309")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveControllerData-request>)))
  "Returns full string definition for message of type '<saveControllerData-request>"
  (cl:format cl:nil "~%string fileName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveControllerData-request)))
  "Returns full string definition for message of type 'saveControllerData-request"
  (cl:format cl:nil "~%string fileName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveControllerData-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'fileName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveControllerData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'saveControllerData-request
    (cl:cons ':fileName (fileName msg))
))
;//! \htmlinclude saveControllerData-response.msg.html

(cl:defclass <saveControllerData-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass saveControllerData-response (<saveControllerData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveControllerData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveControllerData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neuroadaptive_msgs-srv:<saveControllerData-response> is deprecated: use neuroadaptive_msgs-srv:saveControllerData-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <saveControllerData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neuroadaptive_msgs-srv:success-val is deprecated.  Use neuroadaptive_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveControllerData-response>) ostream)
  "Serializes a message object of type '<saveControllerData-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveControllerData-response>) istream)
  "Deserializes a message object of type '<saveControllerData-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveControllerData-response>)))
  "Returns string type for a service object of type '<saveControllerData-response>"
  "neuroadaptive_msgs/saveControllerDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveControllerData-response)))
  "Returns string type for a service object of type 'saveControllerData-response"
  "neuroadaptive_msgs/saveControllerDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveControllerData-response>)))
  "Returns md5sum for a message object of type '<saveControllerData-response>"
  "71f144908c8613a70a312851a43a7309")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveControllerData-response)))
  "Returns md5sum for a message object of type 'saveControllerData-response"
  "71f144908c8613a70a312851a43a7309")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveControllerData-response>)))
  "Returns full string definition for message of type '<saveControllerData-response>"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveControllerData-response)))
  "Returns full string definition for message of type 'saveControllerData-response"
  (cl:format cl:nil "~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveControllerData-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveControllerData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'saveControllerData-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'saveControllerData)))
  'saveControllerData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'saveControllerData)))
  'saveControllerData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveControllerData)))
  "Returns string type for a service object of type '<saveControllerData>"
  "neuroadaptive_msgs/saveControllerData")