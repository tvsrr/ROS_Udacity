# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rtabmap_msgs/GetPlanRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class GetPlanRequest(genpy.Message):
  _md5sum = "93cea387b2aa9245414c000574ff1591"
  _type = "rtabmap_msgs/GetPlanRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """


int32 goal_node

geometry_msgs/PoseStamped goal


float32 tolerance

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['goal_node','goal','tolerance']
  _slot_types = ['int32','geometry_msgs/PoseStamped','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       goal_node,goal,tolerance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetPlanRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.goal_node is None:
        self.goal_node = 0
      if self.goal is None:
        self.goal = geometry_msgs.msg.PoseStamped()
      if self.tolerance is None:
        self.tolerance = 0.
    else:
      self.goal_node = 0
      self.goal = geometry_msgs.msg.PoseStamped()
      self.tolerance = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_i3I().pack(_x.goal_node, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs))
      _x = self.goal.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7df().pack(_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.tolerance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.goal is None:
        self.goal = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.goal_node, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs,) = _get_struct_i3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.goal.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.goal.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 60
      (_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.tolerance,) = _get_struct_7df().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i3I().pack(_x.goal_node, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs))
      _x = self.goal.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7df().pack(_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.tolerance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.goal is None:
        self.goal = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.goal_node, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs,) = _get_struct_i3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.goal.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.goal.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 60
      (_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.tolerance,) = _get_struct_7df().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7df = None
def _get_struct_7df():
    global _struct_7df
    if _struct_7df is None:
        _struct_7df = struct.Struct("<7df")
    return _struct_7df
_struct_i3I = None
def _get_struct_i3I():
    global _struct_i3I
    if _struct_i3I is None:
        _struct_i3I = struct.Struct("<i3I")
    return _struct_i3I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rtabmap_msgs/GetPlanResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import rtabmap_msgs.msg
import std_msgs.msg

class GetPlanResponse(genpy.Message):
  _md5sum = "0412b9858bfcee4b2ee4fbf2f8eb5028"
  _type = "rtabmap_msgs/GetPlanResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Path plan

================================================================================
MSG: rtabmap_msgs/Path

Header header

int32[] nodeIds
geometry_msgs/Pose[] poses


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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['plan']
  _slot_types = ['rtabmap_msgs/Path']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       plan

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetPlanResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.plan is None:
        self.plan = rtabmap_msgs.msg.Path()
    else:
      self.plan = rtabmap_msgs.msg.Path()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs))
      _x = self.plan.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.plan.nodeIds)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.plan.nodeIds))
      length = len(self.plan.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.plan.poses:
        _v1 = val1.position
        _x = _v1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v2 = val1.orientation
        _x = _v2
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.plan is None:
        self.plan = rtabmap_msgs.msg.Path()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.plan.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.plan.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.plan.nodeIds = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.plan.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Pose()
        _v3 = val1.position
        _x = _v3
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v4 = val1.orientation
        _x = _v4
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.plan.poses.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs))
      _x = self.plan.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.plan.nodeIds)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.plan.nodeIds.tostring())
      length = len(self.plan.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.plan.poses:
        _v5 = val1.position
        _x = _v5
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v6 = val1.orientation
        _x = _v6
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.plan is None:
        self.plan = rtabmap_msgs.msg.Path()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.plan.header.seq, _x.plan.header.stamp.secs, _x.plan.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.plan.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.plan.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.plan.nodeIds = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.plan.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Pose()
        _v7 = val1.position
        _x = _v7
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v8 = val1.orientation
        _x = _v8
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.plan.poses.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
class GetPlan(object):
  _type          = 'rtabmap_msgs/GetPlan'
  _md5sum = '11e7a6b05e2b36bab483bec069bf36b9'
  _request_class  = GetPlanRequest
  _response_class = GetPlanResponse
