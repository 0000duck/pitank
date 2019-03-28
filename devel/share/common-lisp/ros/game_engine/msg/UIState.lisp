; Auto-generated. Do not edit!


(cl:in-package game_engine-msg)


;//! \htmlinclude UIState.msg.html

(cl:defclass <UIState> (roslisp-msg-protocol:ros-message)
  ((setupStart
    :reader setupStart
    :initarg :setupStart
    :type cl:boolean
    :initform cl:nil)
   (gameStart
    :reader gameStart
    :initarg :gameStart
    :type cl:boolean
    :initform cl:nil)
   (teamGame
    :reader teamGame
    :initarg :teamGame
    :type cl:boolean
    :initform cl:nil)
   (paused
    :reader paused
    :initarg :paused
    :type cl:boolean
    :initform cl:nil)
   (aiGame
    :reader aiGame
    :initarg :aiGame
    :type cl:boolean
    :initform cl:nil)
   (seconds
    :reader seconds
    :initarg :seconds
    :type cl:integer
    :initform 0))
)

(cl:defclass UIState (<UIState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UIState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UIState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name game_engine-msg:<UIState> is deprecated: use game_engine-msg:UIState instead.")))

(cl:ensure-generic-function 'setupStart-val :lambda-list '(m))
(cl:defmethod setupStart-val ((m <UIState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:setupStart-val is deprecated.  Use game_engine-msg:setupStart instead.")
  (setupStart m))

(cl:ensure-generic-function 'gameStart-val :lambda-list '(m))
(cl:defmethod gameStart-val ((m <UIState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:gameStart-val is deprecated.  Use game_engine-msg:gameStart instead.")
  (gameStart m))

(cl:ensure-generic-function 'teamGame-val :lambda-list '(m))
(cl:defmethod teamGame-val ((m <UIState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:teamGame-val is deprecated.  Use game_engine-msg:teamGame instead.")
  (teamGame m))

(cl:ensure-generic-function 'paused-val :lambda-list '(m))
(cl:defmethod paused-val ((m <UIState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:paused-val is deprecated.  Use game_engine-msg:paused instead.")
  (paused m))

(cl:ensure-generic-function 'aiGame-val :lambda-list '(m))
(cl:defmethod aiGame-val ((m <UIState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:aiGame-val is deprecated.  Use game_engine-msg:aiGame instead.")
  (aiGame m))

(cl:ensure-generic-function 'seconds-val :lambda-list '(m))
(cl:defmethod seconds-val ((m <UIState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:seconds-val is deprecated.  Use game_engine-msg:seconds instead.")
  (seconds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UIState>) ostream)
  "Serializes a message object of type '<UIState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'setupStart) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gameStart) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'teamGame) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'paused) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aiGame) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'seconds)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UIState>) istream)
  "Deserializes a message object of type '<UIState>"
    (cl:setf (cl:slot-value msg 'setupStart) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gameStart) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'teamGame) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'paused) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'aiGame) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'seconds) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UIState>)))
  "Returns string type for a message object of type '<UIState>"
  "game_engine/UIState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UIState)))
  "Returns string type for a message object of type 'UIState"
  "game_engine/UIState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UIState>)))
  "Returns md5sum for a message object of type '<UIState>"
  "3e166d509eed64c7b231607645e715e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UIState)))
  "Returns md5sum for a message object of type 'UIState"
  "3e166d509eed64c7b231607645e715e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UIState>)))
  "Returns full string definition for message of type '<UIState>"
  (cl:format cl:nil "bool setupStart~%bool gameStart~%bool teamGame~%bool paused~%bool aiGame~%int32 seconds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UIState)))
  "Returns full string definition for message of type 'UIState"
  (cl:format cl:nil "bool setupStart~%bool gameStart~%bool teamGame~%bool paused~%bool aiGame~%int32 seconds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UIState>))
  (cl:+ 0
     1
     1
     1
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UIState>))
  "Converts a ROS message object to a list"
  (cl:list 'UIState
    (cl:cons ':setupStart (setupStart msg))
    (cl:cons ':gameStart (gameStart msg))
    (cl:cons ':teamGame (teamGame msg))
    (cl:cons ':paused (paused msg))
    (cl:cons ':aiGame (aiGame msg))
    (cl:cons ':seconds (seconds msg))
))
