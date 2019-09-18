; Auto-generated. Do not edit!


(cl:in-package benjie-msg)


;//! \htmlinclude MarcadorArray.msg.html

(cl:defclass <MarcadorArray> (roslisp-msg-protocol:ros-message)
  ((marcadorArray
    :reader marcadorArray
    :initarg :marcadorArray
    :type (cl:vector benjie-msg:Marcador)
   :initform (cl:make-array 0 :element-type 'benjie-msg:Marcador :initial-element (cl:make-instance 'benjie-msg:Marcador))))
)

(cl:defclass MarcadorArray (<MarcadorArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarcadorArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarcadorArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name benjie-msg:<MarcadorArray> is deprecated: use benjie-msg:MarcadorArray instead.")))

(cl:ensure-generic-function 'marcadorArray-val :lambda-list '(m))
(cl:defmethod marcadorArray-val ((m <MarcadorArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader benjie-msg:marcadorArray-val is deprecated.  Use benjie-msg:marcadorArray instead.")
  (marcadorArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarcadorArray>) ostream)
  "Serializes a message object of type '<MarcadorArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'marcadorArray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'marcadorArray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarcadorArray>) istream)
  "Deserializes a message object of type '<MarcadorArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'marcadorArray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'marcadorArray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'benjie-msg:Marcador))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarcadorArray>)))
  "Returns string type for a message object of type '<MarcadorArray>"
  "benjie/MarcadorArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarcadorArray)))
  "Returns string type for a message object of type 'MarcadorArray"
  "benjie/MarcadorArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarcadorArray>)))
  "Returns md5sum for a message object of type '<MarcadorArray>"
  "d8fc041f5795b2bf581df4a274f1c19a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarcadorArray)))
  "Returns md5sum for a message object of type 'MarcadorArray"
  "d8fc041f5795b2bf581df4a274f1c19a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarcadorArray>)))
  "Returns full string definition for message of type '<MarcadorArray>"
  (cl:format cl:nil "~%~%Marcador[] marcadorArray~%~%================================================================================~%MSG: benjie/Marcador~%int16 id~%int16 cx~%int16 cy~%int16 alpha~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarcadorArray)))
  "Returns full string definition for message of type 'MarcadorArray"
  (cl:format cl:nil "~%~%Marcador[] marcadorArray~%~%================================================================================~%MSG: benjie/Marcador~%int16 id~%int16 cx~%int16 cy~%int16 alpha~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarcadorArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'marcadorArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarcadorArray>))
  "Converts a ROS message object to a list"
  (cl:list 'MarcadorArray
    (cl:cons ':marcadorArray (marcadorArray msg))
))
