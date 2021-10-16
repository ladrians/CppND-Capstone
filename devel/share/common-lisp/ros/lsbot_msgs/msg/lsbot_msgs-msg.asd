
(cl:in-package :asdf)

(defsystem "lsbot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Angle" :depends-on ("_package_Angle"))
    (:file "_package_Angle" :depends-on ("_package"))
    (:file "GoalRotaryServo" :depends-on ("_package_GoalRotaryServo"))
    (:file "_package_GoalRotaryServo" :depends-on ("_package"))
    (:file "StateRotaryServo" :depends-on ("_package_StateRotaryServo"))
    (:file "_package_StateRotaryServo" :depends-on ("_package"))
  ))