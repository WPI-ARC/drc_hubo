
(cl:in-package :asdf)

(defsystem "hackdrc_wave_planner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "HuboWaveTraj" :depends-on ("_package_HuboWaveTraj"))
    (:file "_package_HuboWaveTraj" :depends-on ("_package"))
  ))