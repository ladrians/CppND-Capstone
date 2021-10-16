;; Auto-generated. Do not edit!


(when (boundp 'lsbot_msgs::SetAngle)
  (if (not (find-package "LSBOT_MSGS"))
    (make-package "LSBOT_MSGS"))
  (shadow 'SetAngle (find-package "LSBOT_MSGS")))
(unless (find-package "LSBOT_MSGS::SETANGLE")
  (make-package "LSBOT_MSGS::SETANGLE"))
(unless (find-package "LSBOT_MSGS::SETANGLEREQUEST")
  (make-package "LSBOT_MSGS::SETANGLEREQUEST"))
(unless (find-package "LSBOT_MSGS::SETANGLERESPONSE")
  (make-package "LSBOT_MSGS::SETANGLERESPONSE"))

(in-package "ROS")





(defclass lsbot_msgs::SetAngleRequest
  :super ros::object
  :slots (_angle ))

(defmethod lsbot_msgs::SetAngleRequest
  (:init
   (&key
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass lsbot_msgs::SetAngleResponse
  :super ros::object
  :slots (_success _message ))

(defmethod lsbot_msgs::SetAngleResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass lsbot_msgs::SetAngle
  :super ros::object
  :slots ())

(setf (get lsbot_msgs::SetAngle :md5sum-) "1b7413e2dbca4e7137bc2a82ebcf4118")
(setf (get lsbot_msgs::SetAngle :datatype-) "lsbot_msgs/SetAngle")
(setf (get lsbot_msgs::SetAngle :request) lsbot_msgs::SetAngleRequest)
(setf (get lsbot_msgs::SetAngle :response) lsbot_msgs::SetAngleResponse)

(defmethod lsbot_msgs::SetAngleRequest
  (:response () (instance lsbot_msgs::SetAngleResponse :init)))

(setf (get lsbot_msgs::SetAngleRequest :md5sum-) "1b7413e2dbca4e7137bc2a82ebcf4118")
(setf (get lsbot_msgs::SetAngleRequest :datatype-) "lsbot_msgs/SetAngleRequest")
(setf (get lsbot_msgs::SetAngleRequest :definition-)
      "# Service to set an Angle to the floorscan

float64 angle    # Angle in radians
---
bool success   # True if the call succeeded
string message # Error or informational message
")

(setf (get lsbot_msgs::SetAngleResponse :md5sum-) "1b7413e2dbca4e7137bc2a82ebcf4118")
(setf (get lsbot_msgs::SetAngleResponse :datatype-) "lsbot_msgs/SetAngleResponse")
(setf (get lsbot_msgs::SetAngleResponse :definition-)
      "# Service to set an Angle to the floorscan

float64 angle    # Angle in radians
---
bool success   # True if the call succeeded
string message # Error or informational message
")



(provide :lsbot_msgs/SetAngle "1b7413e2dbca4e7137bc2a82ebcf4118")


