;; Auto-generated. Do not edit!


(when (boundp 'game_engine::UIState)
  (if (not (find-package "GAME_ENGINE"))
    (make-package "GAME_ENGINE"))
  (shadow 'UIState (find-package "GAME_ENGINE")))
(unless (find-package "GAME_ENGINE::UISTATE")
  (make-package "GAME_ENGINE::UISTATE"))

(in-package "ROS")
;;//! \htmlinclude UIState.msg.html


(defclass game_engine::UIState
  :super ros::object
  :slots (_setupStart _gameStart _teamGame _paused _aiGame _seconds ))

(defmethod game_engine::UIState
  (:init
   (&key
    ((:setupStart __setupStart) nil)
    ((:gameStart __gameStart) nil)
    ((:teamGame __teamGame) nil)
    ((:paused __paused) nil)
    ((:aiGame __aiGame) nil)
    ((:seconds __seconds) 0)
    )
   (send-super :init)
   (setq _setupStart __setupStart)
   (setq _gameStart __gameStart)
   (setq _teamGame __teamGame)
   (setq _paused __paused)
   (setq _aiGame __aiGame)
   (setq _seconds (round __seconds))
   self)
  (:setupStart
   (&optional __setupStart)
   (if __setupStart (setq _setupStart __setupStart)) _setupStart)
  (:gameStart
   (&optional __gameStart)
   (if __gameStart (setq _gameStart __gameStart)) _gameStart)
  (:teamGame
   (&optional __teamGame)
   (if __teamGame (setq _teamGame __teamGame)) _teamGame)
  (:paused
   (&optional __paused)
   (if __paused (setq _paused __paused)) _paused)
  (:aiGame
   (&optional __aiGame)
   (if __aiGame (setq _aiGame __aiGame)) _aiGame)
  (:seconds
   (&optional __seconds)
   (if __seconds (setq _seconds __seconds)) _seconds)
  (:serialization-length
   ()
   (+
    ;; bool _setupStart
    1
    ;; bool _gameStart
    1
    ;; bool _teamGame
    1
    ;; bool _paused
    1
    ;; bool _aiGame
    1
    ;; int32 _seconds
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _setupStart
       (if _setupStart (write-byte -1 s) (write-byte 0 s))
     ;; bool _gameStart
       (if _gameStart (write-byte -1 s) (write-byte 0 s))
     ;; bool _teamGame
       (if _teamGame (write-byte -1 s) (write-byte 0 s))
     ;; bool _paused
       (if _paused (write-byte -1 s) (write-byte 0 s))
     ;; bool _aiGame
       (if _aiGame (write-byte -1 s) (write-byte 0 s))
     ;; int32 _seconds
       (write-long _seconds s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _setupStart
     (setq _setupStart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gameStart
     (setq _gameStart (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _teamGame
     (setq _teamGame (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _paused
     (setq _paused (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _aiGame
     (setq _aiGame (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _seconds
     (setq _seconds (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get game_engine::UIState :md5sum-) "3e166d509eed64c7b231607645e715e9")
(setf (get game_engine::UIState :datatype-) "game_engine/UIState")
(setf (get game_engine::UIState :definition-)
      "bool setupStart
bool gameStart
bool teamGame
bool paused
bool aiGame
int32 seconds

")



(provide :game_engine/UIState "3e166d509eed64c7b231607645e715e9")


