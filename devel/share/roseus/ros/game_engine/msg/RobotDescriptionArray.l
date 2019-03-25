;; Auto-generated. Do not edit!


(when (boundp 'game_engine::RobotDescriptionArray)
  (if (not (find-package "GAME_ENGINE"))
    (make-package "GAME_ENGINE"))
  (shadow 'RobotDescriptionArray (find-package "GAME_ENGINE")))
(unless (find-package "GAME_ENGINE::ROBOTDESCRIPTIONARRAY")
  (make-package "GAME_ENGINE::ROBOTDESCRIPTIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude RobotDescriptionArray.msg.html


(defclass game_engine::RobotDescriptionArray
  :super ros::object
  :slots (_robot ))

(defmethod game_engine::RobotDescriptionArray
  (:init
   (&key
    ((:robot __robot) (let (r) (dotimes (i 0) (push (instance game_engine::RobotDescription :init) r)) r))
    )
   (send-super :init)
   (setq _robot __robot)
   self)
  (:robot
   (&rest __robot)
   (if (keywordp (car __robot))
       (send* _robot __robot)
     (progn
       (if __robot (setq _robot (car __robot)))
       _robot)))
  (:serialization-length
   ()
   (+
    ;; game_engine/RobotDescription[] _robot
    (apply #'+ (send-all _robot :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; game_engine/RobotDescription[] _robot
     (write-long (length _robot) s)
     (dolist (elem _robot)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; game_engine/RobotDescription[] _robot
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot (let (r) (dotimes (i n) (push (instance game_engine::RobotDescription :init) r)) r))
     (dolist (elem- _robot)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get game_engine::RobotDescriptionArray :md5sum-) "4200d8a7d47e39131affb929c77aa595")
(setf (get game_engine::RobotDescriptionArray :datatype-) "game_engine/RobotDescriptionArray")
(setf (get game_engine::RobotDescriptionArray :definition-)
      "RobotDescription[] robot

================================================================================
MSG: game_engine/RobotDescription
int8 teamId
int8 tagId
int16 x
int16 y
float32 height
float32 angle
uint8 addr0
uint8 addr1
int8 vel1
int8 vel2
int8 previous_vel
bool collisionFlag
bool threadIsRunning
int8 collisionStateVar
bool autonomous_drive
bool immobilized
int8 damage
int8 kills


")



(provide :game_engine/RobotDescriptionArray "4200d8a7d47e39131affb929c77aa595")


