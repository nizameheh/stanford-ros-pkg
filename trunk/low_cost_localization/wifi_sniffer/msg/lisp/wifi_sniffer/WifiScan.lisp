; Auto-generated. Do not edit!


(in-package wifi_sniffer-msg)


;//! \htmlinclude WifiScan.msg.html

(defclass <WifiScan> (ros-message)
  ((t
    :reader t-val
    :initarg :t
    :type real
    :initform 0)
   (sniffs
    :reader sniffs-val
    :initarg :sniffs
    :type (vector <WifiSniff>)
   :initform (make-array 0 :element-type '<WifiSniff> :initial-element (make-instance '<WifiSniff>))))
)
(defmethod serialize ((msg <WifiScan>) ostream)
  "Serializes a message object of type '<WifiScan>"
  (let ((__sec (floor (slot-value msg 't)))
        (__nsec (round (* 1e9 (- (slot-value msg 't) (floor (slot-value msg 't)))))))
    (write-byte (ldb (byte 8 0) __sec) ostream)
    (write-byte (ldb (byte 8 8) __sec) ostream)
    (write-byte (ldb (byte 8 16) __sec) ostream)
    (write-byte (ldb (byte 8 24) __sec) ostream)
    (write-byte (ldb (byte 8 0) __nsec) ostream)
    (write-byte (ldb (byte 8 8) __nsec) ostream)
    (write-byte (ldb (byte 8 16) __nsec) ostream)
    (write-byte (ldb (byte 8 24) __nsec) ostream))
  (let ((__ros_arr_len (length (slot-value msg 'sniffs))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'sniffs))
)
(defmethod deserialize ((msg <WifiScan>) istream)
  "Deserializes a message object of type '<WifiScan>"
  (let ((__sec 0) (__nsec 0))
    (setf (ldb (byte 8 0) __sec) (read-byte istream))
    (setf (ldb (byte 8 8) __sec) (read-byte istream))
    (setf (ldb (byte 8 16) __sec) (read-byte istream))
    (setf (ldb (byte 8 24) __sec) (read-byte istream))
    (setf (ldb (byte 8 0) __nsec) (read-byte istream))
    (setf (ldb (byte 8 8) __nsec) (read-byte istream))
    (setf (ldb (byte 8 16) __nsec) (read-byte istream))
    (setf (ldb (byte 8 24) __nsec) (read-byte istream))
    (setf (slot-value msg 't) (+ (coerce __sec 'double-float) (/ __nsec 1e9))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'sniffs) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'sniffs)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance '<WifiSniff>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<WifiScan>)))
  "Returns string type for a message object of type '<WifiScan>"
  "wifi_sniffer/WifiScan")
(defmethod md5sum ((type (eql '<WifiScan>)))
  "Returns md5sum for a message object of type '<WifiScan>"
  "cd49c33ac395216be18f78b619e234bd")
(defmethod message-definition ((type (eql '<WifiScan>)))
  "Returns full string definition for message of type '<WifiScan>"
  (format nil "time t~%WifiSniff[] sniffs~%~%================================================================================~%MSG: wifi_sniffer/WifiSniff~%time t~%string mac~%string ssid~%int32 signal_level~%int32 signal_noise~%~%~%"))
(defmethod serialization-length ((msg <WifiScan>))
  (+ 0
     8
     4 (reduce #'+ (slot-value msg 'sniffs) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <WifiScan>))
  "Converts a ROS message object to a list"
  (list '<WifiScan>
    (cons ':t (t-val msg))
    (cons ':sniffs (sniffs-val msg))
))
