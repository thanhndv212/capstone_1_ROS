; Auto-generated. Do not edit!


(cl:in-package core_msgs-msg)


;//! \htmlinclude ball_position.msg.html

(cl:defclass <ball_position> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (img_x
    :reader img_x
    :initarg :img_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (img_y
    :reader img_y
    :initarg :img_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ball_position (<ball_position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ball_position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ball_position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name core_msgs-msg:<ball_position> is deprecated: use core_msgs-msg:ball_position instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ball_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core_msgs-msg:header-val is deprecated.  Use core_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <ball_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core_msgs-msg:size-val is deprecated.  Use core_msgs-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'img_x-val :lambda-list '(m))
(cl:defmethod img_x-val ((m <ball_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core_msgs-msg:img_x-val is deprecated.  Use core_msgs-msg:img_x instead.")
  (img_x m))

(cl:ensure-generic-function 'img_y-val :lambda-list '(m))
(cl:defmethod img_y-val ((m <ball_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core_msgs-msg:img_y-val is deprecated.  Use core_msgs-msg:img_y instead.")
  (img_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ball_position>) ostream)
  "Serializes a message object of type '<ball_position>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'img_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'img_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'img_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'img_y))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ball_position>) istream)
  "Deserializes a message object of type '<ball_position>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'img_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'img_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'img_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'img_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ball_position>)))
  "Returns string type for a message object of type '<ball_position>"
  "core_msgs/ball_position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ball_position)))
  "Returns string type for a message object of type 'ball_position"
  "core_msgs/ball_position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ball_position>)))
  "Returns md5sum for a message object of type '<ball_position>"
  "757a1931a6c8745932637f2569d72982")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ball_position)))
  "Returns md5sum for a message object of type 'ball_position"
  "757a1931a6c8745932637f2569d72982")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ball_position>)))
  "Returns full string definition for message of type '<ball_position>"
  (cl:format cl:nil "Header header~%int32 size~%float32[] img_x~%float32[] img_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ball_position)))
  "Returns full string definition for message of type 'ball_position"
  (cl:format cl:nil "Header header~%int32 size~%float32[] img_x~%float32[] img_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ball_position>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'img_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'img_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ball_position>))
  "Converts a ROS message object to a list"
  (cl:list 'ball_position
    (cl:cons ':header (header msg))
    (cl:cons ':size (size msg))
    (cl:cons ':img_x (img_x msg))
    (cl:cons ':img_y (img_y msg))
))
