;; Auto-generated. Do not edit!


(when (boundp 'core_msgs::ball_position)
  (if (not (find-package "CORE_MSGS"))
    (make-package "CORE_MSGS"))
  (shadow 'ball_position (find-package "CORE_MSGS")))
(unless (find-package "CORE_MSGS::BALL_POSITION")
  (make-package "CORE_MSGS::BALL_POSITION"))

(in-package "ROS")
;;//! \htmlinclude ball_position.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass core_msgs::ball_position
  :super ros::object
  :slots (_header _size _img_x _img_y ))

(defmethod core_msgs::ball_position
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:size __size) 0)
    ((:img_x __img_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:img_y __img_y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _size (round __size))
   (setq _img_x __img_x)
   (setq _img_y __img_y)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:img_x
   (&optional __img_x)
   (if __img_x (setq _img_x __img_x)) _img_x)
  (:img_y
   (&optional __img_y)
   (if __img_y (setq _img_y __img_y)) _img_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _size
    4
    ;; float32[] _img_x
    (* 4    (length _img_x)) 4
    ;; float32[] _img_y
    (* 4    (length _img_y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _size
       (write-long _size s)
     ;; float32[] _img_x
     (write-long (length _img_x) s)
     (dotimes (i (length _img_x))
       (sys::poke (elt _img_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _img_y
     (write-long (length _img_y) s)
     (dotimes (i (length _img_y))
       (sys::poke (elt _img_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _img_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _img_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _img_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _img_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _img_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _img_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get core_msgs::ball_position :md5sum-) "757a1931a6c8745932637f2569d72982")
(setf (get core_msgs::ball_position :datatype-) "core_msgs/ball_position")
(setf (get core_msgs::ball_position :definition-)
      "Header header
int32 size
float32[] img_x
float32[] img_y

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :core_msgs/ball_position "757a1931a6c8745932637f2569d72982")


