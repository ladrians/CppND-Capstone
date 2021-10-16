# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lsbot_msgs/SpecsRotaryServoRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SpecsRotaryServoRequest(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "lsbot_msgs/SpecsRotaryServoRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# device features

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
      super(SpecsRotaryServoRequest, self).__init__(*args, **kwds)

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
    if python3:
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
    if python3:
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
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from lsbot_msgs/SpecsRotaryServoResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SpecsRotaryServoResponse(genpy.Message):
  _md5sum = "c99b39095d63ff4cbaed9c5a6eec7d20"
  _type = "lsbot_msgs/SpecsRotaryServoResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 CONTROL_TYPE_NONE=0
uint8 CONTROL_TYPE_POSITION=1
uint8 CONTROL_TYPE_EFFORT=2
uint8 CONTROL_TYPE_VELOCITY=3
uint8 CONTROL_TYPE_POSITION_VELOCITY=4
uint8 CONTROL_TYPE_POSITION_EFFORT=5
uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
uint8 control_type # rotary servomotor control type

float64 range_min # minimum work range

float64 range_max # maximum work range

float64 precision # angular precision

float64 rated_speed # servomotor speed

float64 reachable_speed # maximum speed

float64 rated_torque # servomotor torque

float64 reachable_torque # peak torque

float64 temperature_range_min # minimum operational temperature

float64 temperature_range_max # maximum operational temperature


"""
  # Pseudo-constants
  CONTROL_TYPE_NONE = 0
  CONTROL_TYPE_POSITION = 1
  CONTROL_TYPE_EFFORT = 2
  CONTROL_TYPE_VELOCITY = 3
  CONTROL_TYPE_POSITION_VELOCITY = 4
  CONTROL_TYPE_POSITION_EFFORT = 5
  CONTROL_TYPE_VELOCITY_EFFORT = 6
  CONTROL_TYPE_POSITION_VELOCITY_EFFORT = 7

  __slots__ = ['control_type','range_min','range_max','precision','rated_speed','reachable_speed','rated_torque','reachable_torque','temperature_range_min','temperature_range_max']
  _slot_types = ['uint8','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       control_type,range_min,range_max,precision,rated_speed,reachable_speed,rated_torque,reachable_torque,temperature_range_min,temperature_range_max

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SpecsRotaryServoResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.control_type is None:
        self.control_type = 0
      if self.range_min is None:
        self.range_min = 0.
      if self.range_max is None:
        self.range_max = 0.
      if self.precision is None:
        self.precision = 0.
      if self.rated_speed is None:
        self.rated_speed = 0.
      if self.reachable_speed is None:
        self.reachable_speed = 0.
      if self.rated_torque is None:
        self.rated_torque = 0.
      if self.reachable_torque is None:
        self.reachable_torque = 0.
      if self.temperature_range_min is None:
        self.temperature_range_min = 0.
      if self.temperature_range_max is None:
        self.temperature_range_max = 0.
    else:
      self.control_type = 0
      self.range_min = 0.
      self.range_max = 0.
      self.precision = 0.
      self.rated_speed = 0.
      self.reachable_speed = 0.
      self.rated_torque = 0.
      self.reachable_torque = 0.
      self.temperature_range_min = 0.
      self.temperature_range_max = 0.

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
      buff.write(_get_struct_B9d().pack(_x.control_type, _x.range_min, _x.range_max, _x.precision, _x.rated_speed, _x.reachable_speed, _x.rated_torque, _x.reachable_torque, _x.temperature_range_min, _x.temperature_range_max))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 73
      (_x.control_type, _x.range_min, _x.range_max, _x.precision, _x.rated_speed, _x.reachable_speed, _x.rated_torque, _x.reachable_torque, _x.temperature_range_min, _x.temperature_range_max,) = _get_struct_B9d().unpack(str[start:end])
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
      buff.write(_get_struct_B9d().pack(_x.control_type, _x.range_min, _x.range_max, _x.precision, _x.rated_speed, _x.reachable_speed, _x.rated_torque, _x.reachable_torque, _x.temperature_range_min, _x.temperature_range_max))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 73
      (_x.control_type, _x.range_min, _x.range_max, _x.precision, _x.rated_speed, _x.reachable_speed, _x.rated_torque, _x.reachable_torque, _x.temperature_range_min, _x.temperature_range_max,) = _get_struct_B9d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B9d = None
def _get_struct_B9d():
    global _struct_B9d
    if _struct_B9d is None:
        _struct_B9d = struct.Struct("<B9d")
    return _struct_B9d
class SpecsRotaryServo(object):
  _type          = 'lsbot_msgs/SpecsRotaryServo'
  _md5sum = 'c99b39095d63ff4cbaed9c5a6eec7d20'
  _request_class  = SpecsRotaryServoRequest
  _response_class = SpecsRotaryServoResponse
