; Auto-generated. Do not edit!


(cl:in-package lsbot_msgs-msg)


;//! \htmlinclude Angle.msg.html

(cl:defclass <Angle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Angle (<Angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-msg:<Angle> is deprecated: use lsbot_msgs-msg:Angle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:header-val is deprecated.  Use lsbot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:status-val is deprecated.  Use lsbot_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:angle-val is deprecated.  Use lsbot_msgs-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Angle>)))
    "Constants for message type '<Angle>"
  '((:UNSTABLE . 0)
    (:STABLE . 1)
    (:UNDEFINED . 2)
    (:RECALIBRATE . -10)
    (:MIN_VALUE . 0.0)
    (:MAX_VALUE . 1.57079632679))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Angle)))
    "Constants for message type 'Angle"
  '((:UNSTABLE . 0)
    (:STABLE . 1)
    (:UNDEFINED . 2)
    (:RECALIBRATE . -10)
    (:MIN_VALUE . 0.0)
    (:MAX_VALUE . 1.57079632679))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Angle>) ostream)
  "Serializes a message object of type '<Angle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Angle>) istream)
  "Deserializes a message object of type '<Angle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Angle>)))
  "Returns string type for a message object of type '<Angle>"
  "lsbot_msgs/Angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Angle)))
  "Returns string type for a message object of type 'Angle"
  "lsbot_msgs/Angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Angle>)))
  "Returns md5sum for a message object of type '<Angle>"
  "01acd77b6666d0b00354cd350705edbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Angle)))
  "Returns md5sum for a message object of type 'Angle"
  "01acd77b6666d0b00354cd350705edbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Angle>)))
  "Returns full string definition for message of type '<Angle>"
  (cl:format cl:nil "# Reported angle by the Floorscan sensor~%~%# Possible Status States~%int16 UNSTABLE = 0      # not yet initialized~%int16 STABLE = 1        # Initialized and working~%int16 UNDEFINED = 2     # Undefined~%int16 RECALIBRATE = -10 # Special angle value to recalibrate~%float64 MIN_VALUE = 0.     # Minimum Value~%float64 MAX_VALUE = 1.57079632679489661923 # Maximum Value~%~%std_msgs/Header header~%int16 status        # Current Status based on the Status States~%float64 angle         # Angle in radians~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Angle)))
  "Returns full string definition for message of type 'Angle"
  (cl:format cl:nil "# Reported angle by the Floorscan sensor~%~%# Possible Status States~%int16 UNSTABLE = 0      # not yet initialized~%int16 STABLE = 1        # Initialized and working~%int16 UNDEFINED = 2     # Undefined~%int16 RECALIBRATE = -10 # Special angle value to recalibrate~%float64 MIN_VALUE = 0.     # Minimum Value~%float64 MAX_VALUE = 1.57079632679489661923 # Maximum Value~%~%std_msgs/Header header~%int16 status        # Current Status based on the Status States~%float64 angle         # Angle in radians~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Angle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Angle>))
  "Converts a ROS message object to a list"
  (cl:list 'Angle
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':angle (angle msg))
))
