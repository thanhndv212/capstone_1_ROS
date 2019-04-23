
(cl:in-package :asdf)

(defsystem "core_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ball_position" :depends-on ("_package_ball_position"))
    (:file "_package_ball_position" :depends-on ("_package"))
    (:file "ball_position_r" :depends-on ("_package_ball_position_r"))
    (:file "_package_ball_position_r" :depends-on ("_package"))
    (:file "markermsg" :depends-on ("_package_markermsg"))
    (:file "_package_markermsg" :depends-on ("_package"))
    (:file "multiarray" :depends-on ("_package_multiarray"))
    (:file "_package_multiarray" :depends-on ("_package"))
  ))