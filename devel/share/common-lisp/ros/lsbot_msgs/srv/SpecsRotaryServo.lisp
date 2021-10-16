; Auto-generated. Do not edit!


(cl:in-package lsbot_msgs-srv)


;//! \htmlinclude SpecsRotaryServo-request.msg.html

(cl:defclass <SpecsRotaryServo-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SpecsRotaryServo-request (<SpecsRotaryServo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpecsRotaryServo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpecsRotaryServo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-srv:<SpecsRotaryServo-request> is deprecated: use lsbot_msgs-srv:SpecsRotaryServo-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpecsRotaryServo-request>) ostream)
  "Serializes a message object of type '<SpecsRotaryServo-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpecsRotaryServo-request>) istream)
  "Deserializes a message object of type '<SpecsRotaryServo-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpecsRotaryServo-request>)))
  "Returns string type for a service object of type '<SpecsRotaryServo-request>"
  "lsbot_msgs/SpecsRotaryServoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpecsRotaryServo-request)))
  "Returns string type for a service object of type 'SpecsRotaryServo-request"
  "lsbot_msgs/SpecsRotaryServoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpecsRotaryServo-request>)))
  "Returns md5sum for a message object of type '<SpecsRotaryServo-request>"
  "c99b39095d63ff4cbaed9c5a6eec7d20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpecsRotaryServo-request)))
  "Returns md5sum for a message object of type 'SpecsRotaryServo-request"
  "c99b39095d63ff4cbaed9c5a6eec7d20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpecsRotaryServo-request>)))
  "Returns full string definition for message of type '<SpecsRotaryServo-request>"
  (cl:format cl:nil "# device features~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpecsRotaryServo-request)))
  "Returns full string definition for message of type 'SpecsRotaryServo-request"
  (cl:format cl:nil "# device features~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpecsRotaryServo-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpecsRotaryServo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SpecsRotaryServo-request
))
;//! \htmlinclude SpecsRotaryServo-response.msg.html

(cl:defclass <SpecsRotaryServo-response> (roslisp-msg-protocol:ros-message)
  ((control_type
    :reader control_type
    :initarg :control_type
    :type cl:fixnum
    :initform 0)
   (range_min
    :reader range_min
    :initarg :range_min
    :type cl:float
    :initform 0.0)
   (range_max
    :reader range_max
    :initarg :range_max
    :type cl:float
    :initform 0.0)
   (precision
    :reader precision
    :initarg :precision
    :type cl:float
    :initform 0.0)
   (rated_speed
    :reader rated_speed
    :initarg :rated_speed
    :type cl:float
    :initform 0.0)
   (reachable_speed
    :reader reachable_speed
    :initarg :reachable_speed
    :type cl:float
    :initform 0.0)
   (rated_torque
    :reader rated_torque
    :initarg :rated_torque
    :type cl:float
    :initform 0.0)
   (reachable_torque
    :reader reachable_torque
    :initarg :reachable_torque
    :type cl:float
    :initform 0.0)
   (temperature_range_min
    :reader temperature_range_min
    :initarg :temperature_range_min
    :type cl:float
    :initform 0.0)
   (temperature_range_max
    :reader temperature_range_max
    :initarg :temperature_range_max
    :type cl:float
    :initform 0.0))
)

(cl:defclass SpecsRotaryServo-response (<SpecsRotaryServo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpecsRotaryServo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpecsRotaryServo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lsbot_msgs-srv:<SpecsRotaryServo-response> is deprecated: use lsbot_msgs-srv:SpecsRotaryServo-response instead.")))

(cl:ensure-generic-function 'control_type-val :lambda-list '(m))
(cl:defmethod control_type-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:control_type-val is deprecated.  Use lsbot_msgs-srv:control_type instead.")
  (control_type m))

(cl:ensure-generic-function 'range_min-val :lambda-list '(m))
(cl:defmethod range_min-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:range_min-val is deprecated.  Use lsbot_msgs-srv:range_min instead.")
  (range_min m))

(cl:ensure-generic-function 'range_max-val :lambda-list '(m))
(cl:defmethod range_max-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:range_max-val is deprecated.  Use lsbot_msgs-srv:range_max instead.")
  (range_max m))

(cl:ensure-generic-function 'precision-val :lambda-list '(m))
(cl:defmethod precision-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:precision-val is deprecated.  Use lsbot_msgs-srv:precision instead.")
  (precision m))

(cl:ensure-generic-function 'rated_speed-val :lambda-list '(m))
(cl:defmethod rated_speed-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:rated_speed-val is deprecated.  Use lsbot_msgs-srv:rated_speed instead.")
  (rated_speed m))

(cl:ensure-generic-function 'reachable_speed-val :lambda-list '(m))
(cl:defmethod reachable_speed-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:reachable_speed-val is deprecated.  Use lsbot_msgs-srv:reachable_speed instead.")
  (reachable_speed m))

(cl:ensure-generic-function 'rated_torque-val :lambda-list '(m))
(cl:defmethod rated_torque-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:rated_torque-val is deprecated.  Use lsbot_msgs-srv:rated_torque instead.")
  (rated_torque m))

(cl:ensure-generic-function 'reachable_torque-val :lambda-list '(m))
(cl:defmethod reachable_torque-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:reachable_torque-val is deprecated.  Use lsbot_msgs-srv:reachable_torque instead.")
  (reachable_torque m))

(cl:ensure-generic-function 'temperature_range_min-val :lambda-list '(m))
(cl:defmethod temperature_range_min-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:temperature_range_min-val is deprecated.  Use lsbot_msgs-srv:temperature_range_min instead.")
  (temperature_range_min m))

(cl:ensure-generic-function 'temperature_range_max-val :lambda-list '(m))
(cl:defmethod temperature_range_max-val ((m <SpecsRotaryServo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lsbot_msgs-srv:temperature_range_max-val is deprecated.  Use lsbot_msgs-srv:temperature_range_max instead.")
  (temperature_range_max m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SpecsRotaryServo-response>)))
    "Constants for message type '<SpecsRotaryServo-response>"
  '((:CONTROL_TYPE_NONE . 0)
    (:CONTROL_TYPE_POSITION . 1)
    (:CONTROL_TYPE_EFFORT . 2)
    (:CONTROL_TYPE_VELOCITY . 3)
    (:CONTROL_TYPE_POSITION_VELOCITY . 4)
    (:CONTROL_TYPE_POSITION_EFFORT . 5)
    (:CONTROL_TYPE_VELOCITY_EFFORT . 6)
    (:CONTROL_TYPE_POSITION_VELOCITY_EFFORT . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SpecsRotaryServo-response)))
    "Constants for message type 'SpecsRotaryServo-response"
  '((:CONTROL_TYPE_NONE . 0)
    (:CONTROL_TYPE_POSITION . 1)
    (:CONTROL_TYPE_EFFORT . 2)
    (:CONTROL_TYPE_VELOCITY . 3)
    (:CONTROL_TYPE_POSITION_VELOCITY . 4)
    (:CONTROL_TYPE_POSITION_EFFORT . 5)
    (:CONTROL_TYPE_VELOCITY_EFFORT . 6)
    (:CONTROL_TYPE_POSITION_VELOCITY_EFFORT . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpecsRotaryServo-response>) ostream)
  "Serializes a message object of type '<SpecsRotaryServo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'range_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'range_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'precision))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rated_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'reachable_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rated_torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'reachable_torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temperature_range_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temperature_range_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpecsRotaryServo-response>) istream)
  "Deserializes a message object of type '<SpecsRotaryServo-response>"
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
    (cl:setf (cl:slot-value msg 'range_min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range_max) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'precision) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rated_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reachable_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rated_torque) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reachable_torque) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_range_min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature_range_max) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpecsRotaryServo-response>)))
  "Returns string type for a service object of type '<SpecsRotaryServo-response>"
  "lsbot_msgs/SpecsRotaryServoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpecsRotaryServo-response)))
  "Returns string type for a service object of type 'SpecsRotaryServo-response"
  "lsbot_msgs/SpecsRotaryServoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpecsRotaryServo-response>)))
  "Returns md5sum for a message object of type '<SpecsRotaryServo-response>"
  "c99b39095d63ff4cbaed9c5a6eec7d20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpecsRotaryServo-response)))
  "Returns md5sum for a message object of type 'SpecsRotaryServo-response"
  "c99b39095d63ff4cbaed9c5a6eec7d20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpecsRotaryServo-response>)))
  "Returns full string definition for message of type '<SpecsRotaryServo-response>"
  (cl:format cl:nil "uint8 CONTROL_TYPE_NONE=0~%uint8 CONTROL_TYPE_POSITION=1~%uint8 CONTROL_TYPE_EFFORT=2~%uint8 CONTROL_TYPE_VELOCITY=3~%uint8 CONTROL_TYPE_POSITION_VELOCITY=4~%uint8 CONTROL_TYPE_POSITION_EFFORT=5~%uint8 CONTROL_TYPE_VELOCITY_EFFORT=6~%uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7~%uint8 control_type # rotary servomotor control type~%~%float64 range_min # minimum work range~%~%float64 range_max # maximum work range~%~%float64 precision # angular precision~%~%float64 rated_speed # servomotor speed~%~%float64 reachable_speed # maximum speed~%~%float64 rated_torque # servomotor torque~%~%float64 reachable_torque # peak torque~%~%float64 temperature_range_min # minimum operational temperature~%~%float64 temperature_range_max # maximum operational temperature~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpecsRotaryServo-response)))
  "Returns full string definition for message of type 'SpecsRotaryServo-response"
  (cl:format cl:nil "uint8 CONTROL_TYPE_NONE=0~%uint8 CONTROL_TYPE_POSITION=1~%uint8 CONTROL_TYPE_EFFORT=2~%uint8 CONTROL_TYPE_VELOCITY=3~%uint8 CONTROL_TYPE_POSITION_VELOCITY=4~%uint8 CONTROL_TYPE_POSITION_EFFORT=5~%uint8 CONTROL_TYPE_VELOCITY_EFFORT=6~%uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7~%uint8 control_type # rotary servomotor control type~%~%float64 range_min # minimum work range~%~%float64 range_max # maximum work range~%~%float64 precision # angular precision~%~%float64 rated_speed # servomotor speed~%~%float64 reachable_speed # maximum speed~%~%float64 rated_torque # servomotor torque~%~%float64 reachable_torque # peak torque~%~%float64 temperature_range_min # minimum operational temperature~%~%float64 temperature_range_max # maximum operational temperature~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpecsRotaryServo-response>))
  (cl:+ 0
     1
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpecsRotaryServo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SpecsRotaryServo-response
    (cl:cons ':control_type (control_type msg))
    (cl:cons ':range_min (range_min msg))
    (cl:cons ':range_max (range_max msg))
    (cl:cons ':precision (precision msg))
    (cl:cons ':rated_speed (rated_speed msg))
    (cl:cons ':reachable_speed (reachable_speed msg))
    (cl:cons ':rated_torque (rated_torque msg))
    (cl:cons ':reachable_torque (reachable_torque msg))
    (cl:cons ':temperature_range_min (temperature_range_min msg))
    (cl:cons ':temperature_range_max (temperature_range_max msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SpecsRotaryServo)))
  'SpecsRotaryServo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SpecsRotaryServo)))
  'SpecsRotaryServo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpecsRotaryServo)))
  "Returns string type for a service object of type '<SpecsRotaryServo>"
  "lsbot_msgs/SpecsRotaryServo")