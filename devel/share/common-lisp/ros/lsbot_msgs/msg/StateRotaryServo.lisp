; Auto-generated. Do not edit!


(cl:in-package lsbot_msgs-msg)


;//! \htmlinclude StateRotaryServo.msg.html

(cl:defclass <StateRotaryServo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal
    :reader goal
    :initarg :goal
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (error
    :reader error
    :initarg :error
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (effort
    :reader effort
    :initarg :effort
    :type cl:float
    :initform 0.0)
   (load
    :reader load
    :initarg :load
    :type cl:float
    :initform 0.0)
   (moving
    :reader moving
    :initarg :moving
    :type cl:boolean
    :initform cl:nil)
   (fault
    :reader fault
    :initarg :fault
    :type cl:fixnum
    :initform 0)
   (control_type
    :reader control_type
    :initarg :control_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StateRotaryServo (<StateRotaryServo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateRotaryServo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateRotaryServo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-msg:<StateRotaryServo> is deprecated: use lsbot_msgs-msg:StateRotaryServo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:header-val is deprecated.  Use lsbot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:goal-val is deprecated.  Use lsbot_msgs-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:position-val is deprecated.  Use lsbot_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:error-val is deprecated.  Use lsbot_msgs-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:velocity-val is deprecated.  Use lsbot_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:effort-val is deprecated.  Use lsbot_msgs-msg:effort instead.")
  (effort m))

(cl:ensure-generic-function 'load-val :lambda-list '(m))
(cl:defmethod load-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:load-val is deprecated.  Use lsbot_msgs-msg:load instead.")
  (load m))

(cl:ensure-generic-function 'moving-val :lambda-list '(m))
(cl:defmethod moving-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:moving-val is deprecated.  Use lsbot_msgs-msg:moving instead.")
  (moving m))

(cl:ensure-generic-function 'fault-val :lambda-list '(m))
(cl:defmethod fault-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:fault-val is deprecated.  Use lsbot_msgs-msg:fault instead.")
  (fault m))

(cl:ensure-generic-function 'control_type-val :lambda-list '(m))
(cl:defmethod control_type-val ((m <StateRotaryServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-msg:control_type-val is deprecated.  Use lsbot_msgs-msg:control_type instead.")
  (control_type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StateRotaryServo>)))
    "Constants for message type '<StateRotaryServo>"
  '((:FAULT_NONE . 0)
    (:FAULT_CURRENT . 1)
    (:FAULT_GENERAL . 2)
    (:CONTROL_TYPE_NONE . 0)
    (:CONTROL_TYPE_POSITION . 1)
    (:CONTROL_TYPE_EFFORT . 2)
    (:CONTROL_TYPE_VELOCITY . 3)
    (:CONTROL_TYPE_POSITION_VELOCITY . 4)
    (:CONTROL_TYPE_POSITION_EFFORT . 5)
    (:CONTROL_TYPE_VELOCITY_EFFORT . 6)
    (:CONTROL_TYPE_POSITION_VELOCITY_EFFORT . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StateRotaryServo)))
    "Constants for message type 'StateRotaryServo"
  '((:FAULT_NONE . 0)
    (:FAULT_CURRENT . 1)
    (:FAULT_GENERAL . 2)
    (:CONTROL_TYPE_NONE . 0)
    (:CONTROL_TYPE_POSITION . 1)
    (:CONTROL_TYPE_EFFORT . 2)
    (:CONTROL_TYPE_VELOCITY . 3)
    (:CONTROL_TYPE_POSITION_VELOCITY . 4)
    (:CONTROL_TYPE_POSITION_EFFORT . 5)
    (:CONTROL_TYPE_VELOCITY_EFFORT . 6)
    (:CONTROL_TYPE_POSITION_VELOCITY_EFFORT . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateRotaryServo>) ostream)
  "Serializes a message object of type '<StateRotaryServo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'moving) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateRotaryServo>) istream)
  "Deserializes a message object of type '<StateRotaryServo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal) (roslisp-utils:decode-double-float-bits bits)))
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
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'load) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'moving) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateRotaryServo>)))
  "Returns string type for a message object of type '<StateRotaryServo>"
  "lsbot_msgs/StateRotaryServo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateRotaryServo)))
  "Returns string type for a message object of type 'StateRotaryServo"
  "lsbot_msgs/StateRotaryServo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateRotaryServo>)))
  "Returns md5sum for a message object of type '<StateRotaryServo>"
  "a7b87918793ebff92fa52e8d24df3ee5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateRotaryServo)))
  "Returns md5sum for a message object of type 'StateRotaryServo"
  "a7b87918793ebff92fa52e8d24df3ee5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateRotaryServo>)))
  "Returns full string definition for message of type '<StateRotaryServo>"
  (cl:format cl:nil "# motor condition and the reason in case of error~%~%std_msgs/Header header # motor state is at this time~%~%float64 goal # commanded position~%~%float64 position # current position encoder~%~%float64 error # difference between current and goal positions~%~%float64 velocity # current velocity~%~%float64 effort # current effort of the actuator~%~%float64 load # load imposed on the motor~%~%bool moving # whether the motor is currently in motion~%~%uint8 FAULT_NONE=0~%uint8 FAULT_CURRENT=1~%uint8 FAULT_GENERAL=2~%uint8 fault # fault cause~%~%uint8 CONTROL_TYPE_NONE=0~%uint8 CONTROL_TYPE_POSITION=1~%uint8 CONTROL_TYPE_EFFORT=2~%uint8 CONTROL_TYPE_VELOCITY=3~%uint8 CONTROL_TYPE_POSITION_VELOCITY=4~%uint8 CONTROL_TYPE_POSITION_EFFORT=5~%uint8 CONTROL_TYPE_VELOCITY_EFFORT=6~%uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7~%uint8 control_type # rotary servomotor control type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateRotaryServo)))
  "Returns full string definition for message of type 'StateRotaryServo"
  (cl:format cl:nil "# motor condition and the reason in case of error~%~%std_msgs/Header header # motor state is at this time~%~%float64 goal # commanded position~%~%float64 position # current position encoder~%~%float64 error # difference between current and goal positions~%~%float64 velocity # current velocity~%~%float64 effort # current effort of the actuator~%~%float64 load # load imposed on the motor~%~%bool moving # whether the motor is currently in motion~%~%uint8 FAULT_NONE=0~%uint8 FAULT_CURRENT=1~%uint8 FAULT_GENERAL=2~%uint8 fault # fault cause~%~%uint8 CONTROL_TYPE_NONE=0~%uint8 CONTROL_TYPE_POSITION=1~%uint8 CONTROL_TYPE_EFFORT=2~%uint8 CONTROL_TYPE_VELOCITY=3~%uint8 CONTROL_TYPE_POSITION_VELOCITY=4~%uint8 CONTROL_TYPE_POSITION_EFFORT=5~%uint8 CONTROL_TYPE_VELOCITY_EFFORT=6~%uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7~%uint8 control_type # rotary servomotor control type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateRotaryServo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateRotaryServo>))
  "Converts a ROS message object to a list"
  (cl:list 'StateRotaryServo
    (cl:cons ':header (header msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':position (position msg))
    (cl:cons ':error (error msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':effort (effort msg))
    (cl:cons ':load (load msg))
    (cl:cons ':moving (moving msg))
    (cl:cons ':fault (fault msg))
    (cl:cons ':control_type (control_type msg))
))
