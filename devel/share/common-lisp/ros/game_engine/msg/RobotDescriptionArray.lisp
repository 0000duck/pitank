; Auto-generated. Do not edit!


(cl:in-package game_engine-msg)


;//! \htmlinclude RobotDescriptionArray.msg.html

(cl:defclass <RobotDescriptionArray> (roslisp-msg-protocol:ros-message)
  ((robot
    :reader robot
    :initarg :robot
    :type (cl:vector game_engine-msg:RobotDescription)
   :initform (cl:make-array 0 :element-type 'game_engine-msg:RobotDescription :initial-element (cl:make-instance 'game_engine-msg:RobotDescription))))
)

(cl:defclass RobotDescriptionArray (<RobotDescriptionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotDescriptionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotDescriptionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name game_engine-msg:<RobotDescriptionArray> is deprecated: use game_engine-msg:RobotDescriptionArray instead.")))

(cl:ensure-generic-function 'robot-val :lambda-list '(m))
(cl:defmethod robot-val ((m <RobotDescriptionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader game_engine-msg:robot-val is deprecated.  Use game_engine-msg:robot instead.")
  (robot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotDescriptionArray>) ostream)
  "Serializes a message object of type '<RobotDescriptionArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robot))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotDescriptionArray>) istream)
  "Deserializes a message object of type '<RobotDescriptionArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robot) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robot)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'game_engine-msg:RobotDescription))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotDescriptionArray>)))
  "Returns string type for a message object of type '<RobotDescriptionArray>"
  "game_engine/RobotDescriptionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotDescriptionArray)))
  "Returns string type for a message object of type 'RobotDescriptionArray"
  "game_engine/RobotDescriptionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotDescriptionArray>)))
  "Returns md5sum for a message object of type '<RobotDescriptionArray>"
  "a10b3056d03141a072c7ffcaaf2ce2ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotDescriptionArray)))
  "Returns md5sum for a message object of type 'RobotDescriptionArray"
  "a10b3056d03141a072c7ffcaaf2ce2ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotDescriptionArray>)))
  "Returns full string definition for message of type '<RobotDescriptionArray>"
  (cl:format cl:nil "RobotDescription[] robot~%~%================================================================================~%MSG: game_engine/RobotDescription~%int8 teamId~%int8 tagId~%int16 x~%int16 y~%float64 height~%float64 angle~%uint8 addr0~%uint8 addr1~%int8 vel1~%int8 vel2~%int8 previous_vel~%bool collisionFlag~%bool threadIsRunning~%int8 collisionStateVar~%bool autonomous_drive~%bool immobilized~%int8 damage~%int8 kills~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotDescriptionArray)))
  "Returns full string definition for message of type 'RobotDescriptionArray"
  (cl:format cl:nil "RobotDescription[] robot~%~%================================================================================~%MSG: game_engine/RobotDescription~%int8 teamId~%int8 tagId~%int16 x~%int16 y~%float64 height~%float64 angle~%uint8 addr0~%uint8 addr1~%int8 vel1~%int8 vel2~%int8 previous_vel~%bool collisionFlag~%bool threadIsRunning~%int8 collisionStateVar~%bool autonomous_drive~%bool immobilized~%int8 damage~%int8 kills~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotDescriptionArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotDescriptionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotDescriptionArray
    (cl:cons ':robot (robot msg))
))
