"""autogenerated by genpy from RiseStereoOdom/freakdescriptor.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class freakdescriptor(genpy.Message):
  _md5sum = "13ff0f1a46fa8500e56e1b85e78ff35d"
  _type = "RiseStereoOdom/freakdescriptor"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32[16] descriptor

"""
  __slots__ = ['descriptor']
  _slot_types = ['int32[16]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       descriptor

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(freakdescriptor, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.descriptor is None:
        self.descriptor = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    else:
      self.descriptor = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

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
      buff.write(_struct_16i.pack(*self.descriptor))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 64
      self.descriptor = _struct_16i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(self.descriptor.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 64
      self.descriptor = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=16)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_16i = struct.Struct("<16i")
