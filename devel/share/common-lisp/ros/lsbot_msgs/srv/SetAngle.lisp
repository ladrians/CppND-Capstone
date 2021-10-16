; Auto-generated. Do not edit!


(cl:in-package lsbot_msgs-srv)


;//! \htmlinclude SetAngle-request.msg.html

(cl:defclass <SetAngle-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetAngle-request (<SetAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-srv:<SetAngle-request> is deprecated: use lsbot_msgs-srv:SetAngle-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <SetAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:angle-val is deprecated.  Use lsbot_msgs-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAngle-request>) ostream)
  "Serializes a message object of type '<SetAngle-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAngle-request>) istream)
  "Deserializes a message object of type '<SetAngle-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAngle-request>)))
  "Returns string type for a service object of type '<SetAngle-request>"
  "lsbot_msgs/SetAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAngle-request)))
  "Returns string type for a service object of type 'SetAngle-request"
  "lsbot_msgs/SetAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAngle-request>)))
  "Returns md5sum for a message object of type '<SetAngle-request>"
  "1b7413e2dbca4e7137bc2a82ebcf4118")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAngle-request)))
  "Returns md5sum for a message object of type 'SetAngle-request"
  "1b7413e2dbca4e7137bc2a82ebcf4118")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAngle-request>)))
  "Returns full string definition for message of type '<SetAngle-request>"
  (cl:format cl:nil "# Service to set an Angle to the floorscan~%~%float64 angle    # Angle in radians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAngle-request)))
  "Returns full string definition for message of type 'SetAngle-request"
  (cl:format cl:nil "# Service to set an Angle to the floorscan~%~%float64 angle    # Angle in radians~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAngle-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAngle-request
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude SetAngle-response.msg.html

(cl:defclass <SetAngle-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetAngle-response (<SetAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-srv:<SetAngle-response> is deprecated: use lsbot_msgs-srv:SetAngle-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:success-val is deprecated.  Use lsbot_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:message-val is deprecated.  Use lsbot_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAngle-response>) ostream)
  "Serializes a message object of type '<SetAngle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAngle-response>) istream)
  "Deserializes a message object of type '<SetAngle-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAngle-response>)))
  "Returns string type for a service object of type '<SetAngle-response>"
  "lsbot_msgs/SetAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAngle-response)))
  "Returns string type for a service object of type 'SetAngle-response"
  "lsbot_msgs/SetAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAngle-response>)))
  "Returns md5sum for a message object of type '<SetAngle-response>"
  "1b7413e2dbca4e7137bc2a82ebcf4118")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAngle-response)))
  "Returns md5sum for a message object of type 'SetAngle-response"
  "1b7413e2dbca4e7137bc2a82ebcf4118")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAngle-response>)))
  "Returns full string definition for message of type '<SetAngle-response>"
  (cl:format cl:nil "bool success   # True if the call succeeded~%string message # Error or informational message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAngle-response)))
  "Returns full string definition for message of type 'SetAngle-response"
  (cl:format cl:nil "bool success   # True if the call succeeded~%string message # Error or informational message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAngle-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAngle-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAngle)))
  'SetAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAngle)))
  'SetAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAngle)))
  "Returns string type for a service object of type '<SetAngle>"
  "lsbot_msgs/SetAngle")