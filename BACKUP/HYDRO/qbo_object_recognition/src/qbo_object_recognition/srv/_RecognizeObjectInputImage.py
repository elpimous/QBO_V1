"""autogenerated by genpy from qbo_object_recognition/RecognizeObjectInputImageRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import sensor_msgs.msg

class RecognizeObjectInputImageRequest(genpy.Message):
  _md5sum = "a290e9e25e4086fd604347d20967f215"
  _type = "qbo_object_recognition/RecognizeObjectInputImageRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """


















sensor_msgs/Image input_image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['input_image']
  _slot_types = ['sensor_msgs/Image']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       input_image

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RecognizeObjectInputImageRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.input_image is None:
        self.input_image = sensor_msgs.msg.Image()
    else:
      self.input_image = sensor_msgs.msg.Image()

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
      buff.write(_struct_3I.pack(_x.input_image.header.seq, _x.input_image.header.stamp.secs, _x.input_image.header.stamp.nsecs))
      _x = self.input_image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.input_image.height, _x.input_image.width))
      _x = self.input_image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.input_image.is_bigendian, _x.input_image.step))
      _x = self.input_image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.input_image is None:
        self.input_image = sensor_msgs.msg.Image()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.input_image.header.seq, _x.input_image.header.stamp.secs, _x.input_image.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input_image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.input_image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.input_image.height, _x.input_image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input_image.encoding = str[start:end].decode('utf-8')
      else:
        self.input_image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.input_image.is_bigendian, _x.input_image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.input_image.data = str[start:end]
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
      _x = self
      buff.write(_struct_3I.pack(_x.input_image.header.seq, _x.input_image.header.stamp.secs, _x.input_image.header.stamp.nsecs))
      _x = self.input_image.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.input_image.height, _x.input_image.width))
      _x = self.input_image.encoding
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_BI.pack(_x.input_image.is_bigendian, _x.input_image.step))
      _x = self.input_image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.input_image is None:
        self.input_image = sensor_msgs.msg.Image()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.input_image.header.seq, _x.input_image.header.stamp.secs, _x.input_image.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input_image.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.input_image.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.input_image.height, _x.input_image.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.input_image.encoding = str[start:end].decode('utf-8')
      else:
        self.input_image.encoding = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.input_image.is_bigendian, _x.input_image.step,) = _struct_BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.input_image.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_BI = struct.Struct("<BI")
"""autogenerated by genpy from qbo_object_recognition/RecognizeObjectInputImageResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class RecognizeObjectInputImageResponse(genpy.Message):
  _md5sum = "20fe0fa539e86f0dcf82db65a3df666b"
  _type = "qbo_object_recognition/RecognizeObjectInputImageResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string object_name
bool recognized


"""
  __slots__ = ['object_name','recognized']
  _slot_types = ['string','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       object_name,recognized

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RecognizeObjectInputImageResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.object_name is None:
        self.object_name = ''
      if self.recognized is None:
        self.recognized = False
    else:
      self.object_name = ''
      self.recognized = False

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
      _x = self.object_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.recognized))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object_name = str[start:end].decode('utf-8')
      else:
        self.object_name = str[start:end]
      start = end
      end += 1
      (self.recognized,) = _struct_B.unpack(str[start:end])
      self.recognized = bool(self.recognized)
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
      _x = self.object_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.recognized))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object_name = str[start:end].decode('utf-8')
      else:
        self.object_name = str[start:end]
      start = end
      end += 1
      (self.recognized,) = _struct_B.unpack(str[start:end])
      self.recognized = bool(self.recognized)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class RecognizeObjectInputImage(object):
  _type          = 'qbo_object_recognition/RecognizeObjectInputImage'
  _md5sum = '71f47bcb0df643ea09ed237fc93b7e61'
  _request_class  = RecognizeObjectInputImageRequest
  _response_class = RecognizeObjectInputImageResponse
