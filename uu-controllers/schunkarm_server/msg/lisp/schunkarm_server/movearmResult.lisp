; Auto-generated. Do not edit!


(in-package schunkarm_server-msg)


;//! \htmlinclude movearmResult.msg.html

(defclass <movearmResult> (ros-message)
  ((position_reached
    :reader position_reached-val
    :initarg :position_reached
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <movearmResult>) ostream)
  "Serializes a message object of type '<movearmResult>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'position_reached) 1 0)) ostream)
)
(defmethod deserialize ((msg <movearmResult>) istream)
  "Deserializes a message object of type '<movearmResult>"
  (setf (slot-value msg 'position_reached) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<movearmResult>)))
  "Returns string type for a message object of type '<movearmResult>"
  "schunkarm_server/movearmResult")
(defmethod md5sum ((type (eql '<movearmResult>)))
  "Returns md5sum for a message object of type '<movearmResult>"
  "3bc153b3a5101bcfc1c76fc2c8373082")
(defmethod message-definition ((type (eql '<movearmResult>)))
  "Returns full string definition for message of type '<movearmResult>"
  (format nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%#Result~%bool position_reached~%~%~%"))
(defmethod serialization-length ((msg <movearmResult>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <movearmResult>))
  "Converts a ROS message object to a list"
  (list '<movearmResult>
    (cons ':position_reached (position_reached-val msg))
))
