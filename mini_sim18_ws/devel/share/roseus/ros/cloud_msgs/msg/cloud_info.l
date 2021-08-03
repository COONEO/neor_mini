;; Auto-generated. Do not edit!


(when (boundp 'cloud_msgs::cloud_info)
  (if (not (find-package "CLOUD_MSGS"))
    (make-package "CLOUD_MSGS"))
  (shadow 'cloud_info (find-package "CLOUD_MSGS")))
(unless (find-package "CLOUD_MSGS::CLOUD_INFO")
  (make-package "CLOUD_MSGS::CLOUD_INFO"))

(in-package "ROS")
;;//! \htmlinclude cloud_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cloud_msgs::cloud_info
  :super ros::object
  :slots (_header _startRingIndex _endRingIndex _startOrientation _endOrientation _orientationDiff _segmentedCloudGroundFlag _segmentedCloudColInd _segmentedCloudRange ))

(defmethod cloud_msgs::cloud_info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:startRingIndex __startRingIndex) (make-array 0 :initial-element 0 :element-type :integer))
    ((:endRingIndex __endRingIndex) (make-array 0 :initial-element 0 :element-type :integer))
    ((:startOrientation __startOrientation) 0.0)
    ((:endOrientation __endOrientation) 0.0)
    ((:orientationDiff __orientationDiff) 0.0)
    ((:segmentedCloudGroundFlag __segmentedCloudGroundFlag) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:segmentedCloudColInd __segmentedCloudColInd) (make-array 0 :initial-element 0 :element-type :integer))
    ((:segmentedCloudRange __segmentedCloudRange) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _startRingIndex __startRingIndex)
   (setq _endRingIndex __endRingIndex)
   (setq _startOrientation (float __startOrientation))
   (setq _endOrientation (float __endOrientation))
   (setq _orientationDiff (float __orientationDiff))
   (setq _segmentedCloudGroundFlag __segmentedCloudGroundFlag)
   (setq _segmentedCloudColInd __segmentedCloudColInd)
   (setq _segmentedCloudRange __segmentedCloudRange)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:startRingIndex
   (&optional __startRingIndex)
   (if __startRingIndex (setq _startRingIndex __startRingIndex)) _startRingIndex)
  (:endRingIndex
   (&optional __endRingIndex)
   (if __endRingIndex (setq _endRingIndex __endRingIndex)) _endRingIndex)
  (:startOrientation
   (&optional __startOrientation)
   (if __startOrientation (setq _startOrientation __startOrientation)) _startOrientation)
  (:endOrientation
   (&optional __endOrientation)
   (if __endOrientation (setq _endOrientation __endOrientation)) _endOrientation)
  (:orientationDiff
   (&optional __orientationDiff)
   (if __orientationDiff (setq _orientationDiff __orientationDiff)) _orientationDiff)
  (:segmentedCloudGroundFlag
   (&optional __segmentedCloudGroundFlag)
   (if __segmentedCloudGroundFlag (setq _segmentedCloudGroundFlag __segmentedCloudGroundFlag)) _segmentedCloudGroundFlag)
  (:segmentedCloudColInd
   (&optional __segmentedCloudColInd)
   (if __segmentedCloudColInd (setq _segmentedCloudColInd __segmentedCloudColInd)) _segmentedCloudColInd)
  (:segmentedCloudRange
   (&optional __segmentedCloudRange)
   (if __segmentedCloudRange (setq _segmentedCloudRange __segmentedCloudRange)) _segmentedCloudRange)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _startRingIndex
    (* 4    (length _startRingIndex)) 4
    ;; int32[] _endRingIndex
    (* 4    (length _endRingIndex)) 4
    ;; float32 _startOrientation
    4
    ;; float32 _endOrientation
    4
    ;; float32 _orientationDiff
    4
    ;; bool[] _segmentedCloudGroundFlag
    (* 1    (length _segmentedCloudGroundFlag)) 4
    ;; uint32[] _segmentedCloudColInd
    (* 4    (length _segmentedCloudColInd)) 4
    ;; float32[] _segmentedCloudRange
    (* 4    (length _segmentedCloudRange)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _startRingIndex
     (write-long (length _startRingIndex) s)
     (dotimes (i (length _startRingIndex))
       (write-long (elt _startRingIndex i) s)
       )
     ;; int32[] _endRingIndex
     (write-long (length _endRingIndex) s)
     (dotimes (i (length _endRingIndex))
       (write-long (elt _endRingIndex i) s)
       )
     ;; float32 _startOrientation
       (sys::poke _startOrientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _endOrientation
       (sys::poke _endOrientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientationDiff
       (sys::poke _orientationDiff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool[] _segmentedCloudGroundFlag
     (write-long (length _segmentedCloudGroundFlag) s)
     (dotimes (i (length _segmentedCloudGroundFlag))
       (if (elt _segmentedCloudGroundFlag i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; uint32[] _segmentedCloudColInd
     (write-long (length _segmentedCloudColInd) s)
     (dotimes (i (length _segmentedCloudColInd))
       (write-long (elt _segmentedCloudColInd i) s)
       )
     ;; float32[] _segmentedCloudRange
     (write-long (length _segmentedCloudRange) s)
     (dotimes (i (length _segmentedCloudRange))
       (sys::poke (elt _segmentedCloudRange i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _startRingIndex
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _startRingIndex (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _startRingIndex i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _endRingIndex
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _endRingIndex (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _endRingIndex i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32 _startOrientation
     (setq _startOrientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _endOrientation
     (setq _endOrientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientationDiff
     (setq _orientationDiff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool[] _segmentedCloudGroundFlag
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _segmentedCloudGroundFlag (make-list n))
     (dotimes (i n)
     (setf (elt _segmentedCloudGroundFlag i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; uint32[] _segmentedCloudColInd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _segmentedCloudColInd (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _segmentedCloudColInd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _segmentedCloudRange
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _segmentedCloudRange (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _segmentedCloudRange i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get cloud_msgs::cloud_info :md5sum-) "af8fdf3af62b4ae75761d0e92aa4cf43")
(setf (get cloud_msgs::cloud_info :datatype-) "cloud_msgs/cloud_info")
(setf (get cloud_msgs::cloud_info :definition-)
      "Header header 

int32[] startRingIndex
int32[] endRingIndex

float32 startOrientation
float32 endOrientation
float32 orientationDiff

bool[]    segmentedCloudGroundFlag # true - ground point, false - other points
uint32[]  segmentedCloudColInd # point column index in range image
float32[] segmentedCloudRange # point range 
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
string frame_id

")



(provide :cloud_msgs/cloud_info "af8fdf3af62b4ae75761d0e92aa4cf43")


