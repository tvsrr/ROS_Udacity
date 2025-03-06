# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rtabmap_msgs/AddLinkRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import rtabmap_msgs.msg

class AddLinkRequest(genpy.Message):
  _md5sum = "050728e3c25a15bf78d487d6aafab152"
  _type = "rtabmap_msgs/AddLinkRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
Link link

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
float64[36] information
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
"""
  __slots__ = ['link']
  _slot_types = ['rtabmap_msgs/Link']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       link

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AddLinkRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.link is None:
        self.link = rtabmap_msgs.msg.Link()
    else:
      self.link = rtabmap_msgs.msg.Link()

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
      buff.write(_get_struct_3i7d().pack(_x.link.fromId, _x.link.toId, _x.link.type, _x.link.transform.translation.x, _x.link.transform.translation.y, _x.link.transform.translation.z, _x.link.transform.rotation.x, _x.link.transform.rotation.y, _x.link.transform.rotation.z, _x.link.transform.rotation.w))
      buff.write(_get_struct_36d().pack(*self.link.information))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.link is None:
        self.link = rtabmap_msgs.msg.Link()
      end = 0
      _x = self
      start = end
      end += 68
      (_x.link.fromId, _x.link.toId, _x.link.type, _x.link.transform.translation.x, _x.link.transform.translation.y, _x.link.transform.translation.z, _x.link.transform.rotation.x, _x.link.transform.rotation.y, _x.link.transform.rotation.z, _x.link.transform.rotation.w,) = _get_struct_3i7d().unpack(str[start:end])
      start = end
      end += 288
      self.link.information = _get_struct_36d().unpack(str[start:end])
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
      buff.write(_get_struct_3i7d().pack(_x.link.fromId, _x.link.toId, _x.link.type, _x.link.transform.translation.x, _x.link.transform.translation.y, _x.link.transform.translation.z, _x.link.transform.rotation.x, _x.link.transform.rotation.y, _x.link.transform.rotation.z, _x.link.transform.rotation.w))
      buff.write(self.link.information.tostring())
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
      if self.link is None:
        self.link = rtabmap_msgs.msg.Link()
      end = 0
      _x = self
      start = end
      end += 68
      (_x.link.fromId, _x.link.toId, _x.link.type, _x.link.transform.translation.x, _x.link.transform.translation.y, _x.link.transform.translation.z, _x.link.transform.rotation.x, _x.link.transform.rotation.y, _x.link.transform.rotation.z, _x.link.transform.rotation.w,) = _get_struct_3i7d().unpack(str[start:end])
      start = end
      end += 288
      self.link.information = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
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
_struct_3i7d = None
def _get_struct_3i7d():
    global _struct_3i7d
    if _struct_3i7d is None:
        _struct_3i7d = struct.Struct("<3i7d")
    return _struct_3i7d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rtabmap_msgs/AddLinkResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AddLinkResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "rtabmap_msgs/AddLinkResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AddLinkResponse, self).__init__(*args, **kwds)

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
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
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
      pass
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
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
class AddLink(object):
  _type          = 'rtabmap_msgs/AddLink'
  _md5sum = '050728e3c25a15bf78d487d6aafab152'
  _request_class  = AddLinkRequest
  _response_class = AddLinkResponse
