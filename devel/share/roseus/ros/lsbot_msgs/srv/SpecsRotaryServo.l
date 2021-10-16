;; Auto-generated. Do not edit!


(when (boundp 'lsbot_msgs::SpecsRotaryServo)
  (if (not (find-package "LSBOT_MSGS"))
    (make-package "LSBOT_MSGS"))
  (shadow 'SpecsRotaryServo (find-package "LSBOT_MSGS")))
(unless (find-package "LSBOT_MSGS::SPECSROTARYSERVO")
  (make-package "LSBOT_MSGS::SPECSROTARYSERVO"))
(unless (find-package "LSBOT_MSGS::SPECSROTARYSERVOREQUEST")
  (make-package "LSBOT_MSGS::SPECSROTARYSERVOREQUEST"))
(unless (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE")
  (make-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))

(in-package "ROS")





(defclass lsbot_msgs::SpecsRotaryServoRequest
  :super ros::object
  :slots ())

(defmethod lsbot_msgs::SpecsRotaryServoRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(intern "*CONTROL_TYPE_NONE*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_NONE* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_NONE* 0)
(intern "*CONTROL_TYPE_POSITION*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_POSITION* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_POSITION* 1)
(intern "*CONTROL_TYPE_EFFORT*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_EFFORT* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_EFFORT* 2)
(intern "*CONTROL_TYPE_VELOCITY*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_VELOCITY* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_VELOCITY* 3)
(intern "*CONTROL_TYPE_POSITION_VELOCITY*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_POSITION_VELOCITY* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_POSITION_VELOCITY* 4)
(intern "*CONTROL_TYPE_POSITION_EFFORT*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_POSITION_EFFORT* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_POSITION_EFFORT* 5)
(intern "*CONTROL_TYPE_VELOCITY_EFFORT*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_VELOCITY_EFFORT* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_VELOCITY_EFFORT* 6)
(intern "*CONTROL_TYPE_POSITION_VELOCITY_EFFORT*" (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(shadow '*CONTROL_TYPE_POSITION_VELOCITY_EFFORT* (find-package "LSBOT_MSGS::SPECSROTARYSERVORESPONSE"))
(defconstant lsbot_msgs::SpecsRotaryServoResponse::*CONTROL_TYPE_POSITION_VELOCITY_EFFORT* 7)
(defclass lsbot_msgs::SpecsRotaryServoResponse
  :super ros::object
  :slots (_control_type _range_min _range_max _precision _rated_speed _reachable_speed _rated_torque _reachable_torque _temperature_range_min _temperature_range_max ))

(defmethod lsbot_msgs::SpecsRotaryServoResponse
  (:init
   (&key
    ((:control_type __control_type) 0)
    ((:range_min __range_min) 0.0)
    ((:range_max __range_max) 0.0)
    ((:precision __precision) 0.0)
    ((:rated_speed __rated_speed) 0.0)
    ((:reachable_speed __reachable_speed) 0.0)
    ((:rated_torque __rated_torque) 0.0)
    ((:reachable_torque __reachable_torque) 0.0)
    ((:temperature_range_min __temperature_range_min) 0.0)
    ((:temperature_range_max __temperature_range_max) 0.0)
    )
   (send-super :init)
   (setq _control_type (round __control_type))
   (setq _range_min (float __range_min))
   (setq _range_max (float __range_max))
   (setq _precision (float __precision))
   (setq _rated_speed (float __rated_speed))
   (setq _reachable_speed (float __reachable_speed))
   (setq _rated_torque (float __rated_torque))
   (setq _reachable_torque (float __reachable_torque))
   (setq _temperature_range_min (float __temperature_range_min))
   (setq _temperature_range_max (float __temperature_range_max))
   self)
  (:control_type
   (&optional __control_type)
   (if __control_type (setq _control_type __control_type)) _control_type)
  (:range_min
   (&optional __range_min)
   (if __range_min (setq _range_min __range_min)) _range_min)
  (:range_max
   (&optional __range_max)
   (if __range_max (setq _range_max __range_max)) _range_max)
  (:precision
   (&optional __precision)
   (if __precision (setq _precision __precision)) _precision)
  (:rated_speed
   (&optional __rated_speed)
   (if __rated_speed (setq _rated_speed __rated_speed)) _rated_speed)
  (:reachable_speed
   (&optional __reachable_speed)
   (if __reachable_speed (setq _reachable_speed __reachable_speed)) _reachable_speed)
  (:rated_torque
   (&optional __rated_torque)
   (if __rated_torque (setq _rated_torque __rated_torque)) _rated_torque)
  (:reachable_torque
   (&optional __reachable_torque)
   (if __reachable_torque (setq _reachable_torque __reachable_torque)) _reachable_torque)
  (:temperature_range_min
   (&optional __temperature_range_min)
   (if __temperature_range_min (setq _temperature_range_min __temperature_range_min)) _temperature_range_min)
  (:temperature_range_max
   (&optional __temperature_range_max)
   (if __temperature_range_max (setq _temperature_range_max __temperature_range_max)) _temperature_range_max)
  (:serialization-length
   ()
   (+
    ;; uint8 _control_type
    1
    ;; float64 _range_min
    8
    ;; float64 _range_max
    8
    ;; float64 _precision
    8
    ;; float64 _rated_speed
    8
    ;; float64 _reachable_speed
    8
    ;; float64 _rated_torque
    8
    ;; float64 _reachable_torque
    8
    ;; float64 _temperature_range_min
    8
    ;; float64 _temperature_range_max
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _control_type
       (write-byte _control_type s)
     ;; float64 _range_min
       (sys::poke _range_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _range_max
       (sys::poke _range_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _precision
       (sys::poke _precision (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rated_speed
       (sys::poke _rated_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _reachable_speed
       (sys::poke _reachable_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rated_torque
       (sys::poke _rated_torque (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _reachable_torque
       (sys::poke _reachable_torque (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temperature_range_min
       (sys::poke _temperature_range_min (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temperature_range_max
       (sys::poke _temperature_range_max (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _control_type
     (setq _control_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _range_min
     (setq _range_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _range_max
     (setq _range_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _precision
     (setq _precision (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rated_speed
     (setq _rated_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _reachable_speed
     (setq _reachable_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rated_torque
     (setq _rated_torque (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _reachable_torque
     (setq _reachable_torque (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temperature_range_min
     (setq _temperature_range_min (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temperature_range_max
     (setq _temperature_range_max (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass lsbot_msgs::SpecsRotaryServo
  :super ros::object
  :slots ())

(setf (get lsbot_msgs::SpecsRotaryServo :md5sum-) "c99b39095d63ff4cbaed9c5a6eec7d20")
(setf (get lsbot_msgs::SpecsRotaryServo :datatype-) "lsbot_msgs/SpecsRotaryServo")
(setf (get lsbot_msgs::SpecsRotaryServo :request) lsbot_msgs::SpecsRotaryServoRequest)
(setf (get lsbot_msgs::SpecsRotaryServo :response) lsbot_msgs::SpecsRotaryServoResponse)

(defmethod lsbot_msgs::SpecsRotaryServoRequest
  (:response () (instance lsbot_msgs::SpecsRotaryServoResponse :init)))

(setf (get lsbot_msgs::SpecsRotaryServoRequest :md5sum-) "c99b39095d63ff4cbaed9c5a6eec7d20")
(setf (get lsbot_msgs::SpecsRotaryServoRequest :datatype-) "lsbot_msgs/SpecsRotaryServoRequest")
(setf (get lsbot_msgs::SpecsRotaryServoRequest :definition-)
      "# device features

---
uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

float64 range_min # minimum work range

float64 range_max # maximum work range

float64 precision # angular precision

float64 rated_speed # servomotor speed

float64 reachable_speed # maximum speed

float64 rated_torque # servomotor torque

float64 reachable_torque # peak torque

float64 temperature_range_min # minimum operational temperature

float64 temperature_range_max # maximum operational temperature


")

(setf (get lsbot_msgs::SpecsRotaryServoResponse :md5sum-) "c99b39095d63ff4cbaed9c5a6eec7d20")
(setf (get lsbot_msgs::SpecsRotaryServoResponse :datatype-) "lsbot_msgs/SpecsRotaryServoResponse")
(setf (get lsbot_msgs::SpecsRotaryServoResponse :definition-)
      "# device features

---
uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

float64 range_min # minimum work range

float64 range_max # maximum work range

float64 precision # angular precision

float64 rated_speed # servomotor speed

float64 reachable_speed # maximum speed

float64 rated_torque # servomotor torque

float64 reachable_torque # peak torque

float64 temperature_range_min # minimum operational temperature

float64 temperature_range_max # maximum operational temperature


")



(provide :lsbot_msgs/SpecsRotaryServo "c99b39095d63ff4cbaed9c5a6eec7d20")


