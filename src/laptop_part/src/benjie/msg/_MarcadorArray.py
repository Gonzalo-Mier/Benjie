"""autogenerated by genpy from benjie/MarcadorArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import benjie.msg

class MarcadorArray(genpy.Message):
  _md5sum = "d8fc041f5795b2bf581df4a274f1c19a"
  _type = "benjie/MarcadorArray"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

Marcador[] marcadorArray

================================================================================
MSG: benjie/Marcador
int16 id
int16 cx
int16 cy
int16 alpha

"""
  __slots__ = ['marcadorArray']
  _slot_types = ['benjie/Marcador[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       marcadorArray

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MarcadorArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.marcadorArray is None:
        self.marcadorArray = []
    else:
      self.marcadorArray = []

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
      length = len(self.marcadorArray)
      buff.write(_struct_I.pack(length))
      for val1 in self.marcadorArray:
        _x = val1
        buff.write(_struct_4h.pack(_x.id, _x.cx, _x.cy, _x.alpha))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.marcadorArray is None:
        self.marcadorArray = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.marcadorArray = []
      for i in range(0, length):
        val1 = benjie.msg.Marcador()
        _x = val1
        start = end
        end += 8
        (_x.id, _x.cx, _x.cy, _x.alpha,) = _struct_4h.unpack(str[start:end])
        self.marcadorArray.append(val1)
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
      length = len(self.marcadorArray)
      buff.write(_struct_I.pack(length))
      for val1 in self.marcadorArray:
        _x = val1
        buff.write(_struct_4h.pack(_x.id, _x.cx, _x.cy, _x.alpha))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.marcadorArray is None:
        self.marcadorArray = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.marcadorArray = []
      for i in range(0, length):
        val1 = benjie.msg.Marcador()
        _x = val1
        start = end
        end += 8
        (_x.id, _x.cx, _x.cy, _x.alpha,) = _struct_4h.unpack(str[start:end])
        self.marcadorArray.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4h = struct.Struct("<4h")