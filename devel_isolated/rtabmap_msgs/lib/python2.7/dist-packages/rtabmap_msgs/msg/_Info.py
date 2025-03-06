# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rtabmap_msgs/Info.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import rtabmap_msgs.msg
import std_msgs.msg

class Info(genpy.Message):
  _md5sum = "656fb5b8dfd8dfd2dc40226f35f2d440"
  _type = "rtabmap_msgs/Info"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """
########################################
# RTAB-Map info with statistics 
########################################

Header header

int32 refId
int32 loopClosureId
int32 proximityDetectionId
int32 landmarkId

geometry_msgs/Transform loopClosureTransform

####
# For statistics...
####
# State (node IDs) of the current Working Memory (including STM)
int32[] wmState

# std::map<int, float> posterior;
int32[] posteriorKeys
float32[] posteriorValues

# std::map<int, float> likelihood;
int32[] likelihoodKeys
float32[] likelihoodValues

# std::map<int, float> rawLikelihood;
int32[] rawLikelihoodKeys
float32[] rawLikelihoodValues

# std::map<int, int> weights;
int32[] weightsKeys
int32[] weightsValues

# std::map<int, std::string> labels;
int32[] labelsKeys
string[] labelsValues

# std::map<std::string, float> stats
string[] statsKeys
float32[] statsValues

# std::vector<int> localPath
int32[] localPath
int32 currentGoalId

# std::vector<int> odomCache
MapGraph odom_cache
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
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

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

================================================================================
MSG: rtabmap_msgs/MapGraph

Header header

##
# /map to /odom transform
# Always identity when the graph is optimized from the latest pose.
##
geometry_msgs/Transform mapToOdom

# The poses
int32[] posesId
geometry_msgs/Pose[] poses

# The links
Link[] links




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
MSG: rtabmap_msgs/Link
#class rtabmap::Link
#{
#   int from;
#   int to;
#   Type type;
#   Transform transform;
#   cv::Mat(6,6,CV_64FC1) information;
#}

int32 fromId
int32 toId
int32 type
geometry_msgs/Transform transform
float64[36] information"""
  __slots__ = ['header','refId','loopClosureId','proximityDetectionId','landmarkId','loopClosureTransform','wmState','posteriorKeys','posteriorValues','likelihoodKeys','likelihoodValues','rawLikelihoodKeys','rawLikelihoodValues','weightsKeys','weightsValues','labelsKeys','labelsValues','statsKeys','statsValues','localPath','currentGoalId','odom_cache']
  _slot_types = ['std_msgs/Header','int32','int32','int32','int32','geometry_msgs/Transform','int32[]','int32[]','float32[]','int32[]','float32[]','int32[]','float32[]','int32[]','int32[]','int32[]','string[]','string[]','float32[]','int32[]','int32','rtabmap_msgs/MapGraph']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,refId,loopClosureId,proximityDetectionId,landmarkId,loopClosureTransform,wmState,posteriorKeys,posteriorValues,likelihoodKeys,likelihoodValues,rawLikelihoodKeys,rawLikelihoodValues,weightsKeys,weightsValues,labelsKeys,labelsValues,statsKeys,statsValues,localPath,currentGoalId,odom_cache

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Info, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.refId is None:
        self.refId = 0
      if self.loopClosureId is None:
        self.loopClosureId = 0
      if self.proximityDetectionId is None:
        self.proximityDetectionId = 0
      if self.landmarkId is None:
        self.landmarkId = 0
      if self.loopClosureTransform is None:
        self.loopClosureTransform = geometry_msgs.msg.Transform()
      if self.wmState is None:
        self.wmState = []
      if self.posteriorKeys is None:
        self.posteriorKeys = []
      if self.posteriorValues is None:
        self.posteriorValues = []
      if self.likelihoodKeys is None:
        self.likelihoodKeys = []
      if self.likelihoodValues is None:
        self.likelihoodValues = []
      if self.rawLikelihoodKeys is None:
        self.rawLikelihoodKeys = []
      if self.rawLikelihoodValues is None:
        self.rawLikelihoodValues = []
      if self.weightsKeys is None:
        self.weightsKeys = []
      if self.weightsValues is None:
        self.weightsValues = []
      if self.labelsKeys is None:
        self.labelsKeys = []
      if self.labelsValues is None:
        self.labelsValues = []
      if self.statsKeys is None:
        self.statsKeys = []
      if self.statsValues is None:
        self.statsValues = []
      if self.localPath is None:
        self.localPath = []
      if self.currentGoalId is None:
        self.currentGoalId = 0
      if self.odom_cache is None:
        self.odom_cache = rtabmap_msgs.msg.MapGraph()
    else:
      self.header = std_msgs.msg.Header()
      self.refId = 0
      self.loopClosureId = 0
      self.proximityDetectionId = 0
      self.landmarkId = 0
      self.loopClosureTransform = geometry_msgs.msg.Transform()
      self.wmState = []
      self.posteriorKeys = []
      self.posteriorValues = []
      self.likelihoodKeys = []
      self.likelihoodValues = []
      self.rawLikelihoodKeys = []
      self.rawLikelihoodValues = []
      self.weightsKeys = []
      self.weightsValues = []
      self.labelsKeys = []
      self.labelsValues = []
      self.statsKeys = []
      self.statsValues = []
      self.localPath = []
      self.currentGoalId = 0
      self.odom_cache = rtabmap_msgs.msg.MapGraph()

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_4i7d().pack(_x.refId, _x.loopClosureId, _x.proximityDetectionId, _x.landmarkId, _x.loopClosureTransform.translation.x, _x.loopClosureTransform.translation.y, _x.loopClosureTransform.translation.z, _x.loopClosureTransform.rotation.x, _x.loopClosureTransform.rotation.y, _x.loopClosureTransform.rotation.z, _x.loopClosureTransform.rotation.w))
      length = len(self.wmState)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.wmState))
      length = len(self.posteriorKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.posteriorKeys))
      length = len(self.posteriorValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.posteriorValues))
      length = len(self.likelihoodKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.likelihoodKeys))
      length = len(self.likelihoodValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.likelihoodValues))
      length = len(self.rawLikelihoodKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.rawLikelihoodKeys))
      length = len(self.rawLikelihoodValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.rawLikelihoodValues))
      length = len(self.weightsKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.weightsKeys))
      length = len(self.weightsValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.weightsValues))
      length = len(self.labelsKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.labelsKeys))
      length = len(self.labelsValues)
      buff.write(_struct_I.pack(length))
      for val1 in self.labelsValues:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      length = len(self.statsKeys)
      buff.write(_struct_I.pack(length))
      for val1 in self.statsKeys:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      length = len(self.statsValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.statsValues))
      length = len(self.localPath)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.localPath))
      _x = self
      buff.write(_get_struct_i3I().pack(_x.currentGoalId, _x.odom_cache.header.seq, _x.odom_cache.header.stamp.secs, _x.odom_cache.header.stamp.nsecs))
      _x = self.odom_cache.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.odom_cache.mapToOdom.translation.x, _x.odom_cache.mapToOdom.translation.y, _x.odom_cache.mapToOdom.translation.z, _x.odom_cache.mapToOdom.rotation.x, _x.odom_cache.mapToOdom.rotation.y, _x.odom_cache.mapToOdom.rotation.z, _x.odom_cache.mapToOdom.rotation.w))
      length = len(self.odom_cache.posesId)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.Struct(pattern).pack(*self.odom_cache.posesId))
      length = len(self.odom_cache.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.odom_cache.poses:
        _v1 = val1.position
        _x = _v1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v2 = val1.orientation
        _x = _v2
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.odom_cache.links)
      buff.write(_struct_I.pack(length))
      for val1 in self.odom_cache.links:
        _x = val1
        buff.write(_get_struct_3i().pack(_x.fromId, _x.toId, _x.type))
        _v3 = val1.transform
        _v4 = _v3.translation
        _x = _v4
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v5 = _v3.rotation
        _x = _v5
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_get_struct_36d().pack(*val1.information))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.loopClosureTransform is None:
        self.loopClosureTransform = geometry_msgs.msg.Transform()
      if self.odom_cache is None:
        self.odom_cache = rtabmap_msgs.msg.MapGraph()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 72
      (_x.refId, _x.loopClosureId, _x.proximityDetectionId, _x.landmarkId, _x.loopClosureTransform.translation.x, _x.loopClosureTransform.translation.y, _x.loopClosureTransform.translation.z, _x.loopClosureTransform.rotation.x, _x.loopClosureTransform.rotation.y, _x.loopClosureTransform.rotation.z, _x.loopClosureTransform.rotation.w,) = _get_struct_4i7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.wmState = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.posteriorKeys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.posteriorValues = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.likelihoodKeys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.likelihoodValues = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.rawLikelihoodKeys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.rawLikelihoodValues = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.weightsKeys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.weightsValues = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.labelsKeys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.labelsValues = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.labelsValues.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.statsKeys = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.statsKeys.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.statsValues = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.localPath = s.unpack(str[start:end])
      _x = self
      start = end
      end += 16
      (_x.currentGoalId, _x.odom_cache.header.seq, _x.odom_cache.header.stamp.secs, _x.odom_cache.header.stamp.nsecs,) = _get_struct_i3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.odom_cache.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.odom_cache.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.odom_cache.mapToOdom.translation.x, _x.odom_cache.mapToOdom.translation.y, _x.odom_cache.mapToOdom.translation.z, _x.odom_cache.mapToOdom.rotation.x, _x.odom_cache.mapToOdom.rotation.y, _x.odom_cache.mapToOdom.rotation.z, _x.odom_cache.mapToOdom.rotation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.odom_cache.posesId = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.odom_cache.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Pose()
        _v6 = val1.position
        _x = _v6
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v7 = val1.orientation
        _x = _v7
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.odom_cache.poses.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.odom_cache.links = []
      for i in range(0, length):
        val1 = rtabmap_msgs.msg.Link()
        _x = val1
        start = end
        end += 12
        (_x.fromId, _x.toId, _x.type,) = _get_struct_3i().unpack(str[start:end])
        _v8 = val1.transform
        _v9 = _v8.translation
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v10 = _v8.rotation
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        start = end
        end += 288
        val1.information = _get_struct_36d().unpack(str[start:end])
        self.odom_cache.links.append(val1)
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_4i7d().pack(_x.refId, _x.loopClosureId, _x.proximityDetectionId, _x.landmarkId, _x.loopClosureTransform.translation.x, _x.loopClosureTransform.translation.y, _x.loopClosureTransform.translation.z, _x.loopClosureTransform.rotation.x, _x.loopClosureTransform.rotation.y, _x.loopClosureTransform.rotation.z, _x.loopClosureTransform.rotation.w))
      length = len(self.wmState)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.wmState.tostring())
      length = len(self.posteriorKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.posteriorKeys.tostring())
      length = len(self.posteriorValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.posteriorValues.tostring())
      length = len(self.likelihoodKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.likelihoodKeys.tostring())
      length = len(self.likelihoodValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.likelihoodValues.tostring())
      length = len(self.rawLikelihoodKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.rawLikelihoodKeys.tostring())
      length = len(self.rawLikelihoodValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.rawLikelihoodValues.tostring())
      length = len(self.weightsKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.weightsKeys.tostring())
      length = len(self.weightsValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.weightsValues.tostring())
      length = len(self.labelsKeys)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.labelsKeys.tostring())
      length = len(self.labelsValues)
      buff.write(_struct_I.pack(length))
      for val1 in self.labelsValues:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      length = len(self.statsKeys)
      buff.write(_struct_I.pack(length))
      for val1 in self.statsKeys:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      length = len(self.statsValues)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.statsValues.tostring())
      length = len(self.localPath)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.localPath.tostring())
      _x = self
      buff.write(_get_struct_i3I().pack(_x.currentGoalId, _x.odom_cache.header.seq, _x.odom_cache.header.stamp.secs, _x.odom_cache.header.stamp.nsecs))
      _x = self.odom_cache.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.odom_cache.mapToOdom.translation.x, _x.odom_cache.mapToOdom.translation.y, _x.odom_cache.mapToOdom.translation.z, _x.odom_cache.mapToOdom.rotation.x, _x.odom_cache.mapToOdom.rotation.y, _x.odom_cache.mapToOdom.rotation.z, _x.odom_cache.mapToOdom.rotation.w))
      length = len(self.odom_cache.posesId)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.odom_cache.posesId.tostring())
      length = len(self.odom_cache.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.odom_cache.poses:
        _v11 = val1.position
        _x = _v11
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v12 = val1.orientation
        _x = _v12
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.odom_cache.links)
      buff.write(_struct_I.pack(length))
      for val1 in self.odom_cache.links:
        _x = val1
        buff.write(_get_struct_3i().pack(_x.fromId, _x.toId, _x.type))
        _v13 = val1.transform
        _v14 = _v13.translation
        _x = _v14
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v15 = _v13.rotation
        _x = _v15
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(val1.information.tostring())
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.loopClosureTransform is None:
        self.loopClosureTransform = geometry_msgs.msg.Transform()
      if self.odom_cache is None:
        self.odom_cache = rtabmap_msgs.msg.MapGraph()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 72
      (_x.refId, _x.loopClosureId, _x.proximityDetectionId, _x.landmarkId, _x.loopClosureTransform.translation.x, _x.loopClosureTransform.translation.y, _x.loopClosureTransform.translation.z, _x.loopClosureTransform.rotation.x, _x.loopClosureTransform.rotation.y, _x.loopClosureTransform.rotation.z, _x.loopClosureTransform.rotation.w,) = _get_struct_4i7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.wmState = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.posteriorKeys = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.posteriorValues = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.likelihoodKeys = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.likelihoodValues = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.rawLikelihoodKeys = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.rawLikelihoodValues = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.weightsKeys = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.weightsValues = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.labelsKeys = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.labelsValues = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.labelsValues.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.statsKeys = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.statsKeys.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.statsValues = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.localPath = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      _x = self
      start = end
      end += 16
      (_x.currentGoalId, _x.odom_cache.header.seq, _x.odom_cache.header.stamp.secs, _x.odom_cache.header.stamp.nsecs,) = _get_struct_i3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.odom_cache.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.odom_cache.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.odom_cache.mapToOdom.translation.x, _x.odom_cache.mapToOdom.translation.y, _x.odom_cache.mapToOdom.translation.z, _x.odom_cache.mapToOdom.rotation.x, _x.odom_cache.mapToOdom.rotation.y, _x.odom_cache.mapToOdom.rotation.z, _x.odom_cache.mapToOdom.rotation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.odom_cache.posesId = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.odom_cache.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Pose()
        _v16 = val1.position
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v17 = val1.orientation
        _x = _v17
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.odom_cache.poses.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.odom_cache.links = []
      for i in range(0, length):
        val1 = rtabmap_msgs.msg.Link()
        _x = val1
        start = end
        end += 12
        (_x.fromId, _x.toId, _x.type,) = _get_struct_3i().unpack(str[start:end])
        _v18 = val1.transform
        _v19 = _v18.translation
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v20 = _v18.rotation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        start = end
        end += 288
        val1.information = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
        self.odom_cache.links.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d
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
_struct_3i = None
def _get_struct_3i():
    global _struct_3i
    if _struct_3i is None:
        _struct_3i = struct.Struct("<3i")
    return _struct_3i
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_4i7d = None
def _get_struct_4i7d():
    global _struct_4i7d
    if _struct_4i7d is None:
        _struct_4i7d = struct.Struct("<4i7d")
    return _struct_4i7d
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_i3I = None
def _get_struct_i3I():
    global _struct_i3I
    if _struct_i3I is None:
        _struct_i3I = struct.Struct("<i3I")
    return _struct_i3I
