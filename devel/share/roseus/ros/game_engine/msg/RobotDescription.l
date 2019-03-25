;; Auto-generated. Do not edit!


(when (boundp 'game_engine::RobotDescription)
  (if (not (find-package "GAME_ENGINE"))
    (make-package "GAME_ENGINE"))
  (shadow 'RobotDescription (find-package "GAME_ENGINE")))
(unless (find-package "GAME_ENGINE::ROBOTDESCRIPTION")
  (make-package "GAME_ENGINE::ROBOTDESCRIPTION"))

(in-package "ROS")
;;//! \htmlinclude RobotDescription.msg.html


(defclass game_engine::RobotDescription
  :super ros::object
  :slots (_teamId _tagId _x _y _height _angle _addr0 _addr1 _vel1 _vel2 _previous_vel _collisionFlag _threadIsRunning _collisionStateVar _autonomous_drive _immobilized _damage _kills ))

(defmethod game_engine::RobotDescription
  (:init
   (&key
    ((:teamId __teamId) 0)
    ((:tagId __tagId) 0)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:height __height) 0.0)
    ((:angle __angle) 0.0)
    ((:addr0 __addr0) 0)
    ((:addr1 __addr1) 0)
    ((:vel1 __vel1) 0)
    ((:vel2 __vel2) 0)
    ((:previous_vel __previous_vel) 0)
    ((:collisionFlag __collisionFlag) nil)
    ((:threadIsRunning __threadIsRunning) nil)
    ((:collisionStateVar __collisionStateVar) 0)
    ((:autonomous_drive __autonomous_drive) nil)
    ((:immobilized __immobilized) nil)
    ((:damage __damage) 0)
    ((:kills __kills) 0)
    )
   (send-super :init)
   (setq _teamId (round __teamId))
   (setq _tagId (round __tagId))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _height (float __height))
   (setq _angle (float __angle))
   (setq _addr0 (round __addr0))
   (setq _addr1 (round __addr1))
   (setq _vel1 (round __vel1))
   (setq _vel2 (round __vel2))
   (setq _previous_vel (round __previous_vel))
   (setq _collisionFlag __collisionFlag)
   (setq _threadIsRunning __threadIsRunning)
   (setq _collisionStateVar (round __collisionStateVar))
   (setq _autonomous_drive __autonomous_drive)
   (setq _immobilized __immobilized)
   (setq _damage (round __damage))
   (setq _kills (round __kills))
   self)
  (:teamId
   (&optional __teamId)
   (if __teamId (setq _teamId __teamId)) _teamId)
  (:tagId
   (&optional __tagId)
   (if __tagId (setq _tagId __tagId)) _tagId)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:addr0
   (&optional __addr0)
   (if __addr0 (setq _addr0 __addr0)) _addr0)
  (:addr1
   (&optional __addr1)
   (if __addr1 (setq _addr1 __addr1)) _addr1)
  (:vel1
   (&optional __vel1)
   (if __vel1 (setq _vel1 __vel1)) _vel1)
  (:vel2
   (&optional __vel2)
   (if __vel2 (setq _vel2 __vel2)) _vel2)
  (:previous_vel
   (&optional __previous_vel)
   (if __previous_vel (setq _previous_vel __previous_vel)) _previous_vel)
  (:collisionFlag
   (&optional __collisionFlag)
   (if __collisionFlag (setq _collisionFlag __collisionFlag)) _collisionFlag)
  (:threadIsRunning
   (&optional __threadIsRunning)
   (if __threadIsRunning (setq _threadIsRunning __threadIsRunning)) _threadIsRunning)
  (:collisionStateVar
   (&optional __collisionStateVar)
   (if __collisionStateVar (setq _collisionStateVar __collisionStateVar)) _collisionStateVar)
  (:autonomous_drive
   (&optional __autonomous_drive)
   (if __autonomous_drive (setq _autonomous_drive __autonomous_drive)) _autonomous_drive)
  (:immobilized
   (&optional __immobilized)
   (if __immobilized (setq _immobilized __immobilized)) _immobilized)
  (:damage
   (&optional __damage)
   (if __damage (setq _damage __damage)) _damage)
  (:kills
   (&optional __kills)
   (if __kills (setq _kills __kills)) _kills)
  (:serialization-length
   ()
   (+
    ;; int8 _teamId
    1
    ;; int8 _tagId
    1
    ;; int16 _x
    2
    ;; int16 _y
    2
    ;; float32 _height
    4
    ;; float32 _angle
    4
    ;; uint8 _addr0
    1
    ;; uint8 _addr1
    1
    ;; int8 _vel1
    1
    ;; int8 _vel2
    1
    ;; int8 _previous_vel
    1
    ;; bool _collisionFlag
    1
    ;; bool _threadIsRunning
    1
    ;; int8 _collisionStateVar
    1
    ;; bool _autonomous_drive
    1
    ;; bool _immobilized
    1
    ;; int8 _damage
    1
    ;; int8 _kills
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _teamId
       (write-byte _teamId s)
     ;; int8 _tagId
       (write-byte _tagId s)
     ;; int16 _x
       (write-word _x s)
     ;; int16 _y
       (write-word _y s)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _addr0
       (write-byte _addr0 s)
     ;; uint8 _addr1
       (write-byte _addr1 s)
     ;; int8 _vel1
       (write-byte _vel1 s)
     ;; int8 _vel2
       (write-byte _vel2 s)
     ;; int8 _previous_vel
       (write-byte _previous_vel s)
     ;; bool _collisionFlag
       (if _collisionFlag (write-byte -1 s) (write-byte 0 s))
     ;; bool _threadIsRunning
       (if _threadIsRunning (write-byte -1 s) (write-byte 0 s))
     ;; int8 _collisionStateVar
       (write-byte _collisionStateVar s)
     ;; bool _autonomous_drive
       (if _autonomous_drive (write-byte -1 s) (write-byte 0 s))
     ;; bool _immobilized
       (if _immobilized (write-byte -1 s) (write-byte 0 s))
     ;; int8 _damage
       (write-byte _damage s)
     ;; int8 _kills
       (write-byte _kills s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _teamId
     (setq _teamId (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _teamId 127) (setq _teamId (- _teamId 256)))
   ;; int8 _tagId
     (setq _tagId (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _tagId 127) (setq _tagId (- _tagId 256)))
   ;; int16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _addr0
     (setq _addr0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _addr1
     (setq _addr1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int8 _vel1
     (setq _vel1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _vel1 127) (setq _vel1 (- _vel1 256)))
   ;; int8 _vel2
     (setq _vel2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _vel2 127) (setq _vel2 (- _vel2 256)))
   ;; int8 _previous_vel
     (setq _previous_vel (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _previous_vel 127) (setq _previous_vel (- _previous_vel 256)))
   ;; bool _collisionFlag
     (setq _collisionFlag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _threadIsRunning
     (setq _threadIsRunning (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _collisionStateVar
     (setq _collisionStateVar (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _collisionStateVar 127) (setq _collisionStateVar (- _collisionStateVar 256)))
   ;; bool _autonomous_drive
     (setq _autonomous_drive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _immobilized
     (setq _immobilized (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _damage
     (setq _damage (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _damage 127) (setq _damage (- _damage 256)))
   ;; int8 _kills
     (setq _kills (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _kills 127) (setq _kills (- _kills 256)))
   ;;
   self)
  )

(setf (get game_engine::RobotDescription :md5sum-) "343c60a2448fde8461e5a48bb9f857c4")
(setf (get game_engine::RobotDescription :datatype-) "game_engine/RobotDescription")
(setf (get game_engine::RobotDescription :definition-)
      "int8 teamId
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



(provide :game_engine/RobotDescription "343c60a2448fde8461e5a48bb9f857c4")


