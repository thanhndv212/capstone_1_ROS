;; Auto-generated. Do not edit!


(when (boundp 'core_msgs::ball_position_r)
  (if (not (find-package "CORE_MSGS"))
    (make-package "CORE_MSGS"))
  (shadow 'ball_position_r (find-package "CORE_MSGS")))
(unless (find-package "CORE_MSGS::BALL_POSITION_R")
  (make-package "CORE_MSGS::BALL_POSITION_R"))

(in-package "ROS")
;;//! \htmlinclude ball_position_r.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass core_msgs::ball_position_r
  :super ros::object
  :slots (_header _size_r _img_x_r _img_y_r ))

(defmethod core_msgs::ball_position_r
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:size_r __size_r) 0)
    ((:img_x_r __img_x_r) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:img_y_r __img_y_r) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _size_r (round __size_r))
   (setq _img_x_r __img_x_r)
   (setq _img_y_r __img_y_r)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:size_r
   (&optional __size_r)
   (if __size_r (setq _size_r __size_r)) _size_r)
  (:img_x_r
   (&optional __img_x_r)
   (if __img_x_r (setq _img_x_r __img_x_r)) _img_x_r)
  (:img_y_r
   (&optional __img_y_r)
   (if __img_y_r (setq _img_y_r __img_y_r)) _img_y_r)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _size_r
    4
    ;; float32[] _img_x_r
    (* 4    (length _img_x_r)) 4
    ;; float32[] _img_y_r
    (* 4    (length _img_y_r)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _size_r
       (write-long _size_r s)
     ;; float32[] _img_x_r
     (write-long (length _img_x_r) s)
     (dotimes (i (length _img_x_r))
       (sys::poke (elt _img_x_r i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _img_y_r
     (write-long (length _img_y_r) s)
     (dotimes (i (length _img_y_r))
       (sys::poke (elt _img_y_r i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _size_r
     (setq _size_r (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _img_x_r
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _img_x_r (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _img_x_r i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _img_y_r
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _img_y_r (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _img_y_r i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get core_msgs::ball_position_r :md5sum-) "726954b4f6c158c4b6d79aac965dd64e")
(setf (get core_msgs::ball_position_r :datatype-) "core_msgs/ball_position_r")
(setf (get core_msgs::ball_position_r :definition-)
      "Header header
int32 size_r
float32[] img_x_r
float32[] img_y_r

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



(provide :core_msgs/ball_position_r "726954b4f6c158c4b6d79aac965dd64e")


