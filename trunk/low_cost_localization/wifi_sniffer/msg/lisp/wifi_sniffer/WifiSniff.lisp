; Auto-generated. Do not edit!


(in-package wifi_sniffer-msg)


;//! \htmlinclude WifiSniff.msg.html

(defclass <WifiSniff> (ros-message)
  ((t
    :reader t-val
    :initarg :t
    :type real
    :initform 0)
   (mac
    :reader mac-val
    :initarg :mac
    :type string
    :initform "")
   (ssid
    :reader ssid-val
    :initarg :ssid
    :type string
    :initform "")
   (signal_level
    :reader signal_level-val
    :initarg :signal_level
    :type integer
    :initform 0)
   (signal_noise
    :reader signal_noise-val
    :initarg :signal_noise
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <WifiSniff>) ostream)
  "Serializes a message object of type '<WifiSniff>"
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
  (let ((__ros_str_len (length (slot-value msg 'mac))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'mac))
  (let ((__ros_str_len (length (slot-value msg 'ssid))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'ssid))
    (write-byte (ldb (byte 8 0) (slot-value msg 'signal_level)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'signal_level)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'signal_level)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'signal_level)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'signal_noise)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'signal_noise)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'signal_noise)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'signal_noise)) ostream)
)
(defmethod deserialize ((msg <WifiSniff>) istream)
  "Deserializes a message object of type '<WifiSniff>"
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
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'mac) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'mac) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'ssid) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'ssid) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'signal_level)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'signal_level)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'signal_level)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'signal_level)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'signal_noise)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'signal_noise)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'signal_noise)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'signal_noise)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<WifiSniff>)))
  "Returns string type for a message object of type '<WifiSniff>"
  "wifi_sniffer/WifiSniff")
(defmethod md5sum ((type (eql '<WifiSniff>)))
  "Returns md5sum for a message object of type '<WifiSniff>"
  "3bed52c7a8a1d7cc0444e57a935d953a")
(defmethod message-definition ((type (eql '<WifiSniff>)))
  "Returns full string definition for message of type '<WifiSniff>"
  (format nil "time t~%string mac~%string ssid~%int32 signal_level~%int32 signal_noise~%~%~%"))
(defmethod serialization-length ((msg <WifiSniff>))
  (+ 0
     8
     4 (length (slot-value msg 'mac))
     4 (length (slot-value msg 'ssid))
     4
     4
))
(defmethod ros-message-to-list ((msg <WifiSniff>))
  "Converts a ROS message object to a list"
  (list '<WifiSniff>
    (cons ':t (t-val msg))
    (cons ':mac (mac-val msg))
    (cons ':ssid (ssid-val msg))
    (cons ':signal_level (signal_level-val msg))
    (cons ':signal_noise (signal_noise-val msg))
))
