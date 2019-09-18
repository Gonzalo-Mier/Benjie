; Auto-generated. Do not edit!


(cl:in-package benjie-msg)


;//! \htmlinclude Marcador.msg.html

(cl:defclass <Marcador> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (cx
    :reader cx
    :initarg :cx
    :type cl:fixnum
    :initform 0)
   (cy
    :reader cy
    :initarg :cy
    :type cl:fixnum
    :initform 0)
   (alpha
    :reader alpha
    :initarg :alpha
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Marcador (<Marcador>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Marcador>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Marcador)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name benjie-msg:<Marcador> is deprecated: use benjie-msg:Marcador instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Marcador>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader benjie-msg:id-val is deprecated.  Use benjie-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'cx-val :lambda-list '(m))
(cl:defmethod cx-val ((m <Marcador>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader benjie-msg:cx-val is deprecated.  Use benjie-msg:cx instead.")
  (cx m))

(cl:ensure-generic-function 'cy-val :lambda-list '(m))
(cl:defmethod cy-val ((m <Marcador>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader benjie-msg:cy-val is deprecated.  Use benjie-msg:cy instead.")
  (cy m))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <Marcador>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader benjie-msg:alpha-val is deprecated.  Use benjie-msg:alpha instead.")
  (alpha m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Marcador>) ostream)
  "Serializes a message object of type '<Marcador>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'alpha)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Marcador>) istream)
  "Deserializes a message object of type '<Marcador>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cx) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cy) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'alpha) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Marcador>)))
  "Returns string type for a message object of type '<Marcador>"
  "benjie/Marcador")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Marcador)))
  "Returns string type for a message object of type 'Marcador"
  "benjie/Marcador")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Marcador>)))
  "Returns md5sum for a message object of type '<Marcador>"
  "b5fac42d34c58923bebd74e8155f7edc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Marcador)))
  "Returns md5sum for a message object of type 'Marcador"
  "b5fac42d34c58923bebd74e8155f7edc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Marcador>)))
  "Returns full string definition for message of type '<Marcador>"
  (cl:format cl:nil "int16 id~%int16 cx~%int16 cy~%int16 alpha~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Marcador)))
  "Returns full string definition for message of type 'Marcador"
  (cl:format cl:nil "int16 id~%int16 cx~%int16 cy~%int16 alpha~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Marcador>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Marcador>))
  "Converts a ROS message object to a list"
  (cl:list 'Marcador
    (cl:cons ':id (id msg))
    (cl:cons ':cx (cx msg))
    (cl:cons ':cy (cy msg))
    (cl:cons ':alpha (alpha msg))
))
