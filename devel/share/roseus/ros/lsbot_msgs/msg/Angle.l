;; Auto-generated. Do not edit!


(when (boundp 'lsbot_msgs::Angle)
  (if (not (find-package "LSBOT_MSGS"))
    (make-package "LSBOT_MSGS"))
  (shadow 'Angle (find-package "LSBOT_MSGS")))
(unless (find-package "LSBOT_MSGS::ANGLE")
  (make-package "LSBOT_MSGS::ANGLE"))

(in-package "ROS")
;;//! \htmlinclude Angle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*UNSTABLE*" (find-package "LSBOT_MSGS::ANGLE"))
(shadow '*UNSTABLE* (find-package "LSBOT_MSGS::ANGLE"))
(defconstant lsbot_msgs::Angle::*UNSTABLE* 0)
(intern "*STABLE*" (find-package "LSBOT_MSGS::ANGLE"))
(shadow '*STABLE* (find-package "LSBOT_MSGS::ANGLE"))
(defconstant lsbot_msgs::Angle::*STABLE* 1)
(intern "*UNDEFINED*" (find-package "LSBOT_MSGS::ANGLE"))
(shadow '*UNDEFINED* (find-package "LSBOT_MSGS::ANGLE"))
(defconstant lsbot_msgs::Angle::*UNDEFINED* 2)
(intern "*RECALIBRATE*" (find-package "LSBOT_MSGS::ANGLE"))
(shadow '*RECALIBRATE* (find-package "LSBOT_MSGS::ANGLE"))
(defconstant lsbot_msgs::Angle::*RECALIBRATE* -10)
(intern "*MIN_VALUE*" (find-package "LSBOT_MSGS::ANGLE"))
(shadow '*MIN_VALUE* (find-package "LSBOT_MSGS::ANGLE"))
(defconstant lsbot_msgs::Angle::*MIN_VALUE* 0.0)
(intern "*MAX_VALUE*" (find-package "LSBOT_MSGS::ANGLE"))
(shadow '*MAX_VALUE* (find-package "LSBOT_MSGS::ANGLE"))
(defconstant lsbot_msgs::Angle::*MAX_VALUE* 1.57079632679)
(defclass lsbot_msgs::Angle
  :super ros::object
  :slots (_header _status _angle ))

(defmethod lsbot_msgs::Angle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) 0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _status (round __status))
   (setq _angle (float __angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _status
    2
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _status
       (write-word _status s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _status
     (setq _status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get lsbot_msgs::Angle :md5sum-) "01acd77b6666d0b00354cd350705edbb")
(setf (get lsbot_msgs::Angle :datatype-) "lsbot_msgs/Angle")
(setf (get lsbot_msgs::Angle :definition-)
      "# Reported angle by the Floorscan sensor

# Possible Status States
int16 UNSTABLE = 0      # not yet initialized
int16 STABLE = 1        # Initialized and working
int16 UNDEFINED = 2     # Undefined
int16 RECALIBRATE = -10 # Special angle value to recalibrate
float64 MIN_VALUE = 0.     # Minimum Value
float64 MAX_VALUE = 1.57079632679489661923 # Maximum Value

std_msgs/Header header
int16 status        # Current Status based on the Status States
float64 angle         # Angle in radians
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



(provide :lsbot_msgs/Angle "01acd77b6666d0b00354cd350705edbb")


