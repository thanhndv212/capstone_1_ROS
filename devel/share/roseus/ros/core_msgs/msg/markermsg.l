;; Auto-generated. Do not edit!


(when (boundp 'core_msgs::markermsg)
  (if (not (find-package "CORE_MSGS"))
    (make-package "CORE_MSGS"))
  (shadow 'markermsg (find-package "CORE_MSGS")))
(unless (find-package "CORE_MSGS::MARKERMSG")
  (make-package "CORE_MSGS::MARKERMSG"))

(in-package "ROS")
;;//! \htmlinclude markermsg.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass core_msgs::markermsg
  :super ros::object
  :slots (_image1_available _image2_available _image1 _image2 _cimage1 _cimage2 ))

(defmethod core_msgs::markermsg
  (:init
   (&key
    ((:image1_available __image1_available) 0)
    ((:image2_available __image2_available) 0)
    ((:image1 __image1) (instance sensor_msgs::Image :init))
    ((:image2 __image2) (instance sensor_msgs::Image :init))
    ((:cimage1 __cimage1) (instance sensor_msgs::CompressedImage :init))
    ((:cimage2 __cimage2) (instance sensor_msgs::CompressedImage :init))
    )
   (send-super :init)
   (setq _image1_available (round __image1_available))
   (setq _image2_available (round __image2_available))
   (setq _image1 __image1)
   (setq _image2 __image2)
   (setq _cimage1 __cimage1)
   (setq _cimage2 __cimage2)
   self)
  (:image1_available
   (&optional __image1_available)
   (if __image1_available (setq _image1_available __image1_available)) _image1_available)
  (:image2_available
   (&optional __image2_available)
   (if __image2_available (setq _image2_available __image2_available)) _image2_available)
  (:image1
   (&rest __image1)
   (if (keywordp (car __image1))
       (send* _image1 __image1)
     (progn
       (if __image1 (setq _image1 (car __image1)))
       _image1)))
  (:image2
   (&rest __image2)
   (if (keywordp (car __image2))
       (send* _image2 __image2)
     (progn
       (if __image2 (setq _image2 (car __image2)))
       _image2)))
  (:cimage1
   (&rest __cimage1)
   (if (keywordp (car __cimage1))
       (send* _cimage1 __cimage1)
     (progn
       (if __cimage1 (setq _cimage1 (car __cimage1)))
       _cimage1)))
  (:cimage2
   (&rest __cimage2)
   (if (keywordp (car __cimage2))
       (send* _cimage2 __cimage2)
     (progn
       (if __cimage2 (setq _cimage2 (car __cimage2)))
       _cimage2)))
  (:serialization-length
   ()
   (+
    ;; int32 _image1_available
    4
    ;; int32 _image2_available
    4
    ;; sensor_msgs/Image _image1
    (send _image1 :serialization-length)
    ;; sensor_msgs/Image _image2
    (send _image2 :serialization-length)
    ;; sensor_msgs/CompressedImage _cimage1
    (send _cimage1 :serialization-length)
    ;; sensor_msgs/CompressedImage _cimage2
    (send _cimage2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _image1_available
       (write-long _image1_available s)
     ;; int32 _image2_available
       (write-long _image2_available s)
     ;; sensor_msgs/Image _image1
       (send _image1 :serialize s)
     ;; sensor_msgs/Image _image2
       (send _image2 :serialize s)
     ;; sensor_msgs/CompressedImage _cimage1
       (send _cimage1 :serialize s)
     ;; sensor_msgs/CompressedImage _cimage2
       (send _cimage2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _image1_available
     (setq _image1_available (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _image2_available
     (setq _image2_available (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; sensor_msgs/Image _image1
     (send _image1 :deserialize buf ptr-) (incf ptr- (send _image1 :serialization-length))
   ;; sensor_msgs/Image _image2
     (send _image2 :deserialize buf ptr-) (incf ptr- (send _image2 :serialization-length))
   ;; sensor_msgs/CompressedImage _cimage1
     (send _cimage1 :deserialize buf ptr-) (incf ptr- (send _cimage1 :serialization-length))
   ;; sensor_msgs/CompressedImage _cimage2
     (send _cimage2 :deserialize buf ptr-) (incf ptr- (send _cimage2 :serialization-length))
   ;;
   self)
  )

(setf (get core_msgs::markermsg :md5sum-) "98f621d5189a43a5c67a336a1b0496cb")
(setf (get core_msgs::markermsg :datatype-) "core_msgs/markermsg")
(setf (get core_msgs::markermsg :definition-)
      "int32 image1_available
int32 image2_available
sensor_msgs/Image image1
sensor_msgs/Image image2
sensor_msgs/CompressedImage cimage1
sensor_msgs/CompressedImage cimage2

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer

")



(provide :core_msgs/markermsg "98f621d5189a43a5c67a336a1b0496cb")


