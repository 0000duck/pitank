; Auto-generated. Do not edit!


(cl:in-package game_engine-msg)


;//! \htmlinclude RobotDescription.msg.html

(cl:defclass <RobotDescription> (roslisp-msg-protocol:ros-message)
  ((teamId
    :reader teamId
    :initarg :teamId
    :type cl:fixnum
    :initform 0)
   (tagId
    :reader tagId
    :initarg :tagId
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (addr0
    :reader addr0
    :initarg :addr0
    :type cl:fixnum
    :initform 0)
   (addr1
    :reader addr1
    :initarg :addr1
    :type cl:fixnum
    :initform 0)
   (vel1
    :reader vel1
    :initarg :vel1
    :type cl:fixnum
    :initform 0)
   (vel2
    :reader vel2
    :initarg :vel2
    :type cl:fixnum
    :initform 0)
   (previous_vel
    :reader previous_vel
    :initarg :previous_vel
    :type cl:fixnum
    :initform 0)
   (collisionFlag
    :reader collisionFlag
    :initarg :collisionFlag
    :type cl:boolean
    :initform cl:nil)
   (threadIsRunning
    :reader threadIsRunning
    :initarg :threadIsRunning
    :type cl:boolean
    :initform cl:nil)
   (collisionStateVar
    :reader collisionStateVar
    :initarg :collisionStateVar
    :type cl:fixnum
    :initform 0)
   (autonomous_drive
    :reader autonomous_drive
    :initarg :autonomous_drive
    :type cl:boolean
    :initform cl:nil)
   (immobilized
    :reader immobilized
    :initarg :immobilized
    :type cl:boolean
    :initform cl:nil)
   (damage
    :reader damage
    :initarg :damage
    :type cl:fixnum
    :initform 0)
   (kills
    :reader kills
    :initarg :kills
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotDescription (<RobotDescription>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotDescription>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotDescription)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name game_engine-msg:<RobotDescription> is deprecated: use game_engine-msg:RobotDescription instead.")))

(cl:ensure-generic-function 'teamId-val :lambda-list '(m))
(cl:defmethod teamId-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:teamId-val is deprecated.  Use game_engine-msg:teamId instead.")
  (teamId m))

(cl:ensure-generic-function 'tagId-val :lambda-list '(m))
(cl:defmethod tagId-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:tagId-val is deprecated.  Use game_engine-msg:tagId instead.")
  (tagId m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:x-val is deprecated.  Use game_engine-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:y-val is deprecated.  Use game_engine-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:height-val is deprecated.  Use game_engine-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:angle-val is deprecated.  Use game_engine-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'addr0-val :lambda-list '(m))
(cl:defmethod addr0-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:addr0-val is deprecated.  Use game_engine-msg:addr0 instead.")
  (addr0 m))

(cl:ensure-generic-function 'addr1-val :lambda-list '(m))
(cl:defmethod addr1-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:addr1-val is deprecated.  Use game_engine-msg:addr1 instead.")
  (addr1 m))

(cl:ensure-generic-function 'vel1-val :lambda-list '(m))
(cl:defmethod vel1-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:vel1-val is deprecated.  Use game_engine-msg:vel1 instead.")
  (vel1 m))

(cl:ensure-generic-function 'vel2-val :lambda-list '(m))
(cl:defmethod vel2-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:vel2-val is deprecated.  Use game_engine-msg:vel2 instead.")
  (vel2 m))

(cl:ensure-generic-function 'previous_vel-val :lambda-list '(m))
(cl:defmethod previous_vel-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:previous_vel-val is deprecated.  Use game_engine-msg:previous_vel instead.")
  (previous_vel m))

(cl:ensure-generic-function 'collisionFlag-val :lambda-list '(m))
(cl:defmethod collisionFlag-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:collisionFlag-val is deprecated.  Use game_engine-msg:collisionFlag instead.")
  (collisionFlag m))

(cl:ensure-generic-function 'threadIsRunning-val :lambda-list '(m))
(cl:defmethod threadIsRunning-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:threadIsRunning-val is deprecated.  Use game_engine-msg:threadIsRunning instead.")
  (threadIsRunning m))

(cl:ensure-generic-function 'collisionStateVar-val :lambda-list '(m))
(cl:defmethod collisionStateVar-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:collisionStateVar-val is deprecated.  Use game_engine-msg:collisionStateVar instead.")
  (collisionStateVar m))

(cl:ensure-generic-function 'autonomous_drive-val :lambda-list '(m))
(cl:defmethod autonomous_drive-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:autonomous_drive-val is deprecated.  Use game_engine-msg:autonomous_drive instead.")
  (autonomous_drive m))

(cl:ensure-generic-function 'immobilized-val :lambda-list '(m))
(cl:defmethod immobilized-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:immobilized-val is deprecated.  Use game_engine-msg:immobilized instead.")
  (immobilized m))

(cl:ensure-generic-function 'damage-val :lambda-list '(m))
(cl:defmethod damage-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:damage-val is deprecated.  Use game_engine-msg:damage instead.")
  (damage m))

(cl:ensure-generic-function 'kills-val :lambda-list '(m))
(cl:defmethod kills-val ((m <RobotDescription>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:kills-val is deprecated.  Use game_engine-msg:kills instead.")
  (kills m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotDescription>) ostream)
  "Serializes a message object of type '<RobotDescription>"
  (cl:let* ((signed (cl:slot-value msg 'teamId)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tagId)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr1)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'vel1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vel2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'previous_vel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'collisionFlag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'threadIsRunning) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'collisionStateVar)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'autonomous_drive) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'immobilized) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'damage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'kills)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotDescription>) istream)
  "Deserializes a message object of type '<RobotDescription>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'teamId) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tagId) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr1)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'previous_vel) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'collisionFlag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'threadIsRunning) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'collisionStateVar) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'autonomous_drive) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'immobilized) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'damage) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kills) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotDescription>)))
  "Returns string type for a message object of type '<RobotDescription>"
  "game_engine/RobotDescription")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotDescription)))
  "Returns string type for a message object of type 'RobotDescription"
  "game_engine/RobotDescription")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotDescription>)))
  "Returns md5sum for a message object of type '<RobotDescription>"
  "35b35bffdcea4af3c10704857437163d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotDescription)))
  "Returns md5sum for a message object of type 'RobotDescription"
  "35b35bffdcea4af3c10704857437163d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotDescription>)))
  "Returns full string definition for message of type '<RobotDescription>"
  (cl:format cl:nil "int8 teamId~%int8 tagId~%int16 x~%int16 y~%float64 height~%float64 angle~%uint8 addr0~%uint8 addr1~%int8 vel1~%int8 vel2~%int8 previous_vel~%bool collisionFlag~%bool threadIsRunning~%int8 collisionStateVar~%bool autonomous_drive~%bool immobilized~%int8 damage~%int8 kills~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotDescription)))
  "Returns full string definition for message of type 'RobotDescription"
  (cl:format cl:nil "int8 teamId~%int8 tagId~%int16 x~%int16 y~%float64 height~%float64 angle~%uint8 addr0~%uint8 addr1~%int8 vel1~%int8 vel2~%int8 previous_vel~%bool collisionFlag~%bool threadIsRunning~%int8 collisionStateVar~%bool autonomous_drive~%bool immobilized~%int8 damage~%int8 kills~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotDescription>))
  (cl:+ 0
     1
     1
     2
     2
     8
     8
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotDescription>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotDescription
    (cl:cons ':teamId (teamId msg))
    (cl:cons ':tagId (tagId msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':height (height msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':addr0 (addr0 msg))
    (cl:cons ':addr1 (addr1 msg))
    (cl:cons ':vel1 (vel1 msg))
    (cl:cons ':vel2 (vel2 msg))
    (cl:cons ':previous_vel (previous_vel msg))
    (cl:cons ':collisionFlag (collisionFlag msg))
    (cl:cons ':threadIsRunning (threadIsRunning msg))
    (cl:cons ':collisionStateVar (collisionStateVar msg))
    (cl:cons ':autonomous_drive (autonomous_drive msg))
    (cl:cons ':immobilized (immobilized msg))
    (cl:cons ':damage (damage msg))
    (cl:cons ':kills (kills msg))
))
