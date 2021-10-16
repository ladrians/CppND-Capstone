; Auto-generated. Do not edit!


(cl:in-package lsbot_msgs-msg)


;//! \htmlinclude GoalRotaryServo.msg.html

(cl:defclass <GoalRotaryServo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (control_type
    :reader control_type
    :initarg :control_type
    :type cl:fixnum
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (effort
    :reader effort
    :initarg :effort
    :type cl:float
    :initform 0.0))
)

(cl:defclass GoalRotaryServo (<GoalRotaryServo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalRotaryServo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalRotaryServo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-msg:<GoalRotaryServo> is deprecated: use lsbot_msgs-msg:GoalRotaryServo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GoalRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:header-val is deprecated.  Use lsbot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'control_type-val :lambda-list '(m))
(cl:defmethod control_type-val ((m <GoalRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:control_type-val is deprecated.  Use lsbot_msgs-msg:control_type instead.")
  (control_type m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <GoalRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:position-val is deprecated.  Use lsbot_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <GoalRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:velocity-val is deprecated.  Use lsbot_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <GoalRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:acceleration-val is deprecated.  Use lsbot_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <GoalRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:effort-val is deprecated.  Use lsbot_msgs-msg:effort instead.")
  (effort m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GoalRotaryServo>)))
    "Constants for message type '<GoalRotaryServo>"
  '((:CONTROL_TYPE_NONE . 0)
    (:CONTROL_TYPE_POSITION . 1)
    (:CONTROL_TYPE_EFFORT . 2)
    (:CONTROL_TYPE_VELOCITY . 3)
    (:CONTROL_TYPE_POSITION_VELOCITY . 4)
    (:CONTROL_TYPE_POSITION_EFFORT . 5)
    (:CONTROL_TYPE_VELOCITY_EFFORT . 6)
    (:CONTROL_TYPE_POSITION_VELOCITY_EFFORT . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GoalRotaryServo)))
    "Constants for message type 'GoalRotaryServo"
  '((:CONTROL_TYPE_NONE . 0)
    (:CONTROL_TYPE_POSITION . 1)
    (:CONTROL_TYPE_EFFORT . 2)
    (:CONTROL_TYPE_VELOCITY . 3)
    (:CONTROL_TYPE_POSITION_VELOCITY . 4)
    (:CONTROL_TYPE_POSITION_EFFORT . 5)
    (:CONTROL_TYPE_VELOCITY_EFFORT . 6)
    (:CONTROL_TYPE_POSITION_VELOCITY_EFFORT . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalRotaryServo>) ostream)
  "Serializes a message object of type '<GoalRotaryServo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalRotaryServo>) istream)
  "Deserializes a message object of type '<GoalRotaryServo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalRotaryServo>)))
  "Returns string type for a message object of type '<GoalRotaryServo>"
  "lsbot_msgs/GoalRotaryServo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalRotaryServo)))
  "Returns string type for a message object of type 'GoalRotaryServo"
  "lsbot_msgs/GoalRotaryServo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalRotaryServo>)))
  "Returns md5sum for a message object of type '<GoalRotaryServo>"
  "d7c284381d46455f889dcdafb56f5389")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalRotaryServo)))
  "Returns md5sum for a message object of type 'GoalRotaryServo"
  "d7c284381d46455f889dcdafb56f5389")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalRotaryServo>)))
  "Returns full string definition for message of type '<GoalRotaryServo>"
  (cl:format cl:nil "# control the position, velocity or/and effort~%~%std_msgs/Header header~%~%uint8 CONTROL_TYPE_NONE=0~%uint8 CONTROL_TYPE_POSITION=1~%uint8 CONTROL_TYPE_EFFORT=2~%uint8 CONTROL_TYPE_VELOCITY=3~%uint8 CONTROL_TYPE_POSITION_VELOCITY=4~%uint8 CONTROL_TYPE_POSITION_EFFORT=5~%uint8 CONTROL_TYPE_VELOCITY_EFFORT=6~%uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7~%uint8 control_type # rotary servomotor control type~%~%float64 position # goal position~%~%float32 velocity # movement velocity~%~%float32 acceleration # movement acceleration~%~%float32 effort # movement torque~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalRotaryServo)))
  "Returns full string definition for message of type 'GoalRotaryServo"
  (cl:format cl:nil "# control the position, velocity or/and effort~%~%std_msgs/Header header~%~%uint8 CONTROL_TYPE_NONE=0~%uint8 CONTROL_TYPE_POSITION=1~%uint8 CONTROL_TYPE_EFFORT=2~%uint8 CONTROL_TYPE_VELOCITY=3~%uint8 CONTROL_TYPE_POSITION_VELOCITY=4~%uint8 CONTROL_TYPE_POSITION_EFFORT=5~%uint8 CONTROL_TYPE_VELOCITY_EFFORT=6~%uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7~%uint8 control_type # rotary servomotor control type~%~%float64 position # goal position~%~%float32 velocity # movement velocity~%~%float32 acceleration # movement acceleration~%~%float32 effort # movement torque~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalRotaryServo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalRotaryServo>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalRotaryServo
    (cl:cons ':header (header msg))
    (cl:cons ':control_type (control_type msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':effort (effort msg))
))
