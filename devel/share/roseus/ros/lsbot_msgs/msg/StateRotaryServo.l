;; Auto-generated. Do not edit!


(when (boundp 'lsbot_msgs::StateRotaryServo)
  (if (not (find-package "LSBOT_MSGS"))
    (make-package "LSBOT_MSGS"))
  (shadow 'StateRotaryServo (find-package "LSBOT_MSGS")))
(unless (find-package "LSBOT_MSGS::STATEROTARYSERVO")
  (make-package "LSBOT_MSGS::STATEROTARYSERVO"))

(in-package "ROS")
;;//! \htmlinclude StateRotaryServo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*FAULT_NONE*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*FAULT_NONE* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*FAULT_NONE* 0)
(intern "*FAULT_CURRENT*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*FAULT_CURRENT* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*FAULT_CURRENT* 1)
(intern "*FAULT_GENERAL*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*FAULT_GENERAL* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*FAULT_GENERAL* 2)
(intern "*CONTROL_TYPE_NONE*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_NONE* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_NONE* 0)
(intern "*CONTROL_TYPE_POSITION*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_POSITION* 1)
(intern "*CONTROL_TYPE_EFFORT*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_EFFORT* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_EFFORT* 2)
(intern "*CONTROL_TYPE_VELOCITY*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_VELOCITY* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_VELOCITY* 3)
(intern "*CONTROL_TYPE_POSITION_VELOCITY*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION_VELOCITY* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_POSITION_VELOCITY* 4)
(intern "*CONTROL_TYPE_POSITION_EFFORT*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION_EFFORT* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_POSITION_EFFORT* 5)
(intern "*CONTROL_TYPE_VELOCITY_EFFORT*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_VELOCITY_EFFORT* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_VELOCITY_EFFORT* 6)
(intern "*CONTROL_TYPE_POSITION_VELOCITY_EFFORT*" (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(shadow '*CONTROL_TYPE_POSITION_VELOCITY_EFFORT* (find-package "LSBOT_MSGS::STATEROTARYSERVO"))
(defconstant lsbot_msgs::StateRotaryServo::*CONTROL_TYPE_POSITION_VELOCITY_EFFORT* 7)
(defclass lsbot_msgs::StateRotaryServo
  :super ros::object
  :slots (_header _goal _position _error _velocity _effort _load _moving _fault _control_type ))

(defmethod lsbot_msgs::StateRotaryServo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goal __goal) 0.0)
    ((:position __position) 0.0)
    ((:error __error) 0.0)
    ((:velocity __velocity) 0.0)
    ((:effort __effort) 0.0)
    ((:load __load) 0.0)
    ((:moving __moving) nil)
    ((:fault __fault) 0)
    ((:control_type __control_type) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _goal (float __goal))
   (setq _position (float __position))
   (setq _error (float __error))
   (setq _velocity (float __velocity))
   (setq _effort (float __effort))
   (setq _load (float __load))
   (setq _moving __moving)
   (setq _fault (round __fault))
   (setq _control_type (round __control_type))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:moving
   (&optional __moving)
   (if __moving (setq _moving __moving)) _moving)
  (:fault
   (&optional __fault)
   (if __fault (setq _fault __fault)) _fault)
  (:control_type
   (&optional __control_type)
   (if __control_type (setq _control_type __control_type)) _control_type)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _goal
    8
    ;; float64 _position
    8
    ;; float64 _error
    8
    ;; float64 _velocity
    8
    ;; float64 _effort
    8
    ;; float64 _load
    8
    ;; bool _moving
    1
    ;; uint8 _fault
    1
    ;; uint8 _control_type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _goal
       (sys::poke _goal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _error
       (sys::poke _error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _effort
       (sys::poke _effort (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _load
       (sys::poke _load (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _moving
       (if _moving (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _fault
       (write-byte _fault s)
     ;; uint8 _control_type
       (write-byte _control_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _goal
     (setq _goal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _error
     (setq _error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _effort
     (setq _effort (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _load
     (setq _load (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _moving
     (setq _moving (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _fault
     (setq _fault (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _control_type
     (setq _control_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get lsbot_msgs::StateRotaryServo :md5sum-) "a7b87918793ebff92fa52e8d24df3ee5")
(setf (get lsbot_msgs::StateRotaryServo :datatype-) "lsbot_msgs/StateRotaryServo")
(setf (get lsbot_msgs::StateRotaryServo :definition-)
      "# motor condition and the reason in case of error

std_msgs/Header header # motor state is at this time

float64 goal # commanded position

float64 position # current position encoder

float64 error # difference between current and goal positions

float64 velocity # current velocity

float64 effort # current effort of the actuator

float64 load # load imposed on the motor

bool moving # whether the motor is currently in motion

uint8 FAULT_NONE=0
uint8 FAULT_CURRENT=1
uint8 FAULT_GENERAL=2
uint8 fault # fault cause

uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

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



(provide :lsbot_msgs/StateRotaryServo "a7b87918793ebff92fa52e8d24df3ee5")


