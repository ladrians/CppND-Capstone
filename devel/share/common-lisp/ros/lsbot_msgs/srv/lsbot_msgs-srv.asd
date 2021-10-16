
(cl:in-package :asdf)

(defsystem "lsbot_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetAngle" :depends-on ("_package_SetAngle"))
    (:file "_package_SetAngle" :depends-on ("_package"))
    (:file "SpecsRotaryServo" :depends-on ("_package_SpecsRotaryServo"))
    (:file "_package_SpecsRotaryServo" :depends-on ("_package"))
  ))