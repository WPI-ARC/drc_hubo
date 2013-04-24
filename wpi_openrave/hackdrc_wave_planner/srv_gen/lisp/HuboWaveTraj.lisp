; Auto-generated. Do not edit!


(cl:in-package hackdrc_wave_planner-srv)


;//! \htmlinclude HuboWaveTraj-request.msg.html

(cl:defclass <HuboWaveTraj-request> (roslisp-msg-protocol:ros-message)
  ((Request
    :reader Request
    :initarg :Request
    :type std_msgs-msg:Empty
    :initform (cl:make-instance 'std_msgs-msg:Empty)))
)

(cl:defclass HuboWaveTraj-request (<HuboWaveTraj-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HuboWaveTraj-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HuboWaveTraj-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hackdrc_wave_planner-srv:<HuboWaveTraj-request> is deprecated: use hackdrc_wave_planner-srv:HuboWaveTraj-request instead.")))

(cl:ensure-generic-function 'Request-val :lambda-list '(m))
(cl:defmethod Request-val ((m <HuboWaveTraj-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hackdrc_wave_planner-srv:Request-val is deprecated.  Use hackdrc_wave_planner-srv:Request instead.")
  (Request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HuboWaveTraj-request>) ostream)
  "Serializes a message object of type '<HuboWaveTraj-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Request) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HuboWaveTraj-request>) istream)
  "Deserializes a message object of type '<HuboWaveTraj-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Request) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HuboWaveTraj-request>)))
  "Returns string type for a service object of type '<HuboWaveTraj-request>"
  "hackdrc_wave_planner/HuboWaveTrajRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HuboWaveTraj-request)))
  "Returns string type for a service object of type 'HuboWaveTraj-request"
  "hackdrc_wave_planner/HuboWaveTrajRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HuboWaveTraj-request>)))
  "Returns md5sum for a message object of type '<HuboWaveTraj-request>"
  "483580a368d8caaee0fa0f82c8e2e571")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HuboWaveTraj-request)))
  "Returns md5sum for a message object of type 'HuboWaveTraj-request"
  "483580a368d8caaee0fa0f82c8e2e571")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HuboWaveTraj-request>)))
  "Returns full string definition for message of type '<HuboWaveTraj-request>"
  (cl:format cl:nil "std_msgs/Empty Request~%~%================================================================================~%MSG: std_msgs/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HuboWaveTraj-request)))
  "Returns full string definition for message of type 'HuboWaveTraj-request"
  (cl:format cl:nil "std_msgs/Empty Request~%~%================================================================================~%MSG: std_msgs/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HuboWaveTraj-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HuboWaveTraj-request>))
  "Converts a ROS message object to a list"
  (cl:list 'HuboWaveTraj-request
    (cl:cons ':Request (Request msg))
))
;//! \htmlinclude HuboWaveTraj-response.msg.html

(cl:defclass <HuboWaveTraj-response> (roslisp-msg-protocol:ros-message)
  ((Response
    :reader Response
    :initarg :Response
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass HuboWaveTraj-response (<HuboWaveTraj-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HuboWaveTraj-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HuboWaveTraj-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hackdrc_wave_planner-srv:<HuboWaveTraj-response> is deprecated: use hackdrc_wave_planner-srv:HuboWaveTraj-response instead.")))

(cl:ensure-generic-function 'Response-val :lambda-list '(m))
(cl:defmethod Response-val ((m <HuboWaveTraj-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hackdrc_wave_planner-srv:Response-val is deprecated.  Use hackdrc_wave_planner-srv:Response instead.")
  (Response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HuboWaveTraj-response>) ostream)
  "Serializes a message object of type '<HuboWaveTraj-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Response) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HuboWaveTraj-response>) istream)
  "Deserializes a message object of type '<HuboWaveTraj-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Response) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HuboWaveTraj-response>)))
  "Returns string type for a service object of type '<HuboWaveTraj-response>"
  "hackdrc_wave_planner/HuboWaveTrajResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HuboWaveTraj-response)))
  "Returns string type for a service object of type 'HuboWaveTraj-response"
  "hackdrc_wave_planner/HuboWaveTrajResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HuboWaveTraj-response>)))
  "Returns md5sum for a message object of type '<HuboWaveTraj-response>"
  "483580a368d8caaee0fa0f82c8e2e571")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HuboWaveTraj-response)))
  "Returns md5sum for a message object of type 'HuboWaveTraj-response"
  "483580a368d8caaee0fa0f82c8e2e571")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HuboWaveTraj-response>)))
  "Returns full string definition for message of type '<HuboWaveTraj-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory Response~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HuboWaveTraj-response)))
  "Returns full string definition for message of type 'HuboWaveTraj-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory Response~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HuboWaveTraj-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HuboWaveTraj-response>))
  "Converts a ROS message object to a list"
  (cl:list 'HuboWaveTraj-response
    (cl:cons ':Response (Response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'HuboWaveTraj)))
  'HuboWaveTraj-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'HuboWaveTraj)))
  'HuboWaveTraj-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HuboWaveTraj)))
  "Returns string type for a service object of type '<HuboWaveTraj>"
  "hackdrc_wave_planner/HuboWaveTraj")