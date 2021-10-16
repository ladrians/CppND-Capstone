;; Auto-generated. Do not edit!


(when (boundp 'lsbot_msgs::GoalRotaryServo)
  (if (not (find-package "LSBOT_MSGS"))
    (make-package "LSBOT_MSGS"))
  (shadow 'GoalRotaryServo (find-package "LSBOT_MSGS")))
(unless (find-package "LSBOT_MSGS::GOALROTARYSERVO")
  (make-package "LSBOT_MSGS::GOALROTARYSERVO"))

(in-package "ROS")
;;//! \htmlinclude GoalRotaryServo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CONTROL_TYPE_NONE*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_NONE* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_NONE* 0)
(intern "*CONTROL_TYPE_POSITION*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_POSITION* 1)
(intern "*CONTROL_TYPE_EFFORT*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_EFFORT* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_EFFORT* 2)
(intern "*CONTROL_TYPE_VELOCITY*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_VELOCITY* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_VELOCITY* 3)
(intern "*CONTROL_TYPE_POSITION_VELOCITY*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION_VELOCITY* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_POSITION_VELOCITY* 4)
(intern "*CONTROL_TYPE_POSITION_EFFORT*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION_EFFORT* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_POSITION_EFFORT* 5)
(intern "*CONTROL_TYPE_VELOCITY_EFFORT*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_VELOCITY_EFFORT* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_VELOCITY_EFFORT* 6)
(intern "*CONTROL_TYPE_POSITION_VELOCITY_EFFORT*" (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION_VELOCITY_EFFORT* (find-package "LSBOT_MSGS::GOALROTARYSERVO"))
(defconstant lsbot_msgs::GoalRotaryServo::*CONTROL_TYPE_POSITION_VELOCITY_EFFORT* 7)
(defclass lsbot_msgs::GoalRotaryServo
  :super ros::object
  :slots (_header _control_type _position _velocity _acceleration _effort ))

(defmethod lsbot_msgs::GoalRotaryServo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:control_type __control_type) 0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:acceleration __acceleration) 0.0)
    ((:effort __effort) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _control_type (round __control_type))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _acceleration (float __acceleration))
   (setq _effort (float __effort))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:control_type
   (&optional __control_type)
   (if __control_type (setq _control_type __control_type)) _control_type)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _control_type
    1
    ;; float64 _position
    8
    ;; float32 _velocity
    4
    ;; float32 _acceleration
    4
    ;; float32 _effort
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _control_type
       (write-byte _control_type s)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _effort
       (sys::poke _effort (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _control_type
     (setq _control_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _effort
     (setq _effort (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lsbot_msgs::GoalRotaryServo :md5sum-) "d7c284381d46455f889dcdafb56f5389")
(setf (get lsbot_msgs::GoalRotaryServo :datatype-) "lsbot_msgs/GoalRotaryServo")
(setf (get lsbot_msgs::GoalRotaryServo :definition-)
      "# control the position, velocity or/and effort

std_msgs/Header header

uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

float64 position # goal position

float32 velocity # movement velocity

float32 acceleration # movement acceleration

float32 effort # movement torque


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :lsbot_msgs/GoalRotaryServo "d7c284381d46455f889dcdafb56f5389")


