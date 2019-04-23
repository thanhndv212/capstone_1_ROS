;; Auto-generated. Do not edit!


(when (boundp 'core_msgs::multiarray)
  (if (not (find-package "CORE_MSGS"))
    (make-package "CORE_MSGS"))
  (shadow 'multiarray (find-package "CORE_MSGS")))
(unless (find-package "CORE_MSGS::MULTIARRAY")
  (make-package "CORE_MSGS::MULTIARRAY"))

(in-package "ROS")
;;//! \htmlinclude multiarray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass core_msgs::multiarray
  :super ros::object
  :slots (_header _data _cols _rows ))

(defmethod core_msgs::multiarray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:cols __cols) 0)
    ((:rows __rows) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   (setq _cols (round __cols))
   (setq _rows (round __rows))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:cols
   (&optional __cols)
   (if __cols (setq _cols __cols)) _cols)
  (:rows
   (&optional __rows)
   (if __rows (setq _rows __rows)) _rows)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8[] _data
    (* 1    (length _data)) 4
    ;; int32 _cols
    4
    ;; int32 _rows
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-byte (elt _data i) s)
       )
     ;; int32 _cols
       (write-long _cols s)
     ;; int32 _rows
       (write-long _rows s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _data i) 127) (setf (elt _data i) (- (elt _data i) 256)))
     ))
   ;; int32 _cols
     (setq _cols (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _rows
     (setq _rows (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get core_msgs::multiarray :md5sum-) "5f8105e16e9e9d9f45d4acb38630a37d")
(setf (get core_msgs::multiarray :datatype-) "core_msgs/multiarray")
(setf (get core_msgs::multiarray :definition-)
      "Header header
int8[] data
int32 cols
int32 rows

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



(provide :core_msgs/multiarray "5f8105e16e9e9d9f45d4acb38630a37d")


