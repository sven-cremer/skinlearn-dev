"""autogenerated by genpy from neuroadaptive_msgs/controllerParam.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class controllerParam(genpy.Message):
  _md5sum = "af3621078a88b07f1ac5f3bd11b79f17"
  _type = "neuroadaptive_msgs/controllerParam"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# NN Params
float64 kappa
float64 Kv
float64 lambda
float64 Kz
float64 Zb
float64 F
float64 G
int64 inParams
int64 outParams
int64 hiddenNodes
int64 errorParams
float64 feedForwardForce
float64 nn_ON

# Cart params
float64 cartPos_Kp_x
float64 cartPos_Kp_y
float64 cartPos_Kp_z
float64 cartPos_Kd_x
float64 cartPos_Kd_y
float64 cartPos_Kd_z

float64 cartRot_Kp_x
float64 cartRot_Kp_y
float64 cartRot_Kp_z
float64 cartRot_Kd_x
float64 cartRot_Kd_y
float64 cartRot_Kd_z

bool useCurrentCartPose
bool useNullspacePose

float64 cartIniX    
float64 cartIniY    
float64 cartIniZ
float64 cartIniRoll 
float64 cartIniPitch
float64 cartIniYaw  

float64 cartDesX    
float64 cartDesY    
float64 cartDesZ    
float64 cartDesRoll 
float64 cartDesPitch
float64 cartDesYaw  

# Ref Model Params
float64 m
float64 d
float64 k

float64 task_mA
float64 task_mB
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
  __slots__ = ['header','kappa','Kv','lambda_','Kz','Zb','F','G','inParams','outParams','hiddenNodes','errorParams','feedForwardForce','nn_ON','cartPos_Kp_x','cartPos_Kp_y','cartPos_Kp_z','cartPos_Kd_x','cartPos_Kd_y','cartPos_Kd_z','cartRot_Kp_x','cartRot_Kp_y','cartRot_Kp_z','cartRot_Kd_x','cartRot_Kd_y','cartRot_Kd_z','useCurrentCartPose','useNullspacePose','cartIniX','cartIniY','cartIniZ','cartIniRoll','cartIniPitch','cartIniYaw','cartDesX','cartDesY','cartDesZ','cartDesRoll','cartDesPitch','cartDesYaw','m','d','k','task_mA','task_mB']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','float64','float64','int64','int64','int64','int64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','bool','bool','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,kappa,Kv,lambda_,Kz,Zb,F,G,inParams,outParams,hiddenNodes,errorParams,feedForwardForce,nn_ON,cartPos_Kp_x,cartPos_Kp_y,cartPos_Kp_z,cartPos_Kd_x,cartPos_Kd_y,cartPos_Kd_z,cartRot_Kp_x,cartRot_Kp_y,cartRot_Kp_z,cartRot_Kd_x,cartRot_Kd_y,cartRot_Kd_z,useCurrentCartPose,useNullspacePose,cartIniX,cartIniY,cartIniZ,cartIniRoll,cartIniPitch,cartIniYaw,cartDesX,cartDesY,cartDesZ,cartDesRoll,cartDesPitch,cartDesYaw,m,d,k,task_mA,task_mB

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(controllerParam, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.kappa is None:
        self.kappa = 0.
      if self.Kv is None:
        self.Kv = 0.
      if self.lambda_ is None:
        self.lambda_ = 0.
      if self.Kz is None:
        self.Kz = 0.
      if self.Zb is None:
        self.Zb = 0.
      if self.F is None:
        self.F = 0.
      if self.G is None:
        self.G = 0.
      if self.inParams is None:
        self.inParams = 0
      if self.outParams is None:
        self.outParams = 0
      if self.hiddenNodes is None:
        self.hiddenNodes = 0
      if self.errorParams is None:
        self.errorParams = 0
      if self.feedForwardForce is None:
        self.feedForwardForce = 0.
      if self.nn_ON is None:
        self.nn_ON = 0.
      if self.cartPos_Kp_x is None:
        self.cartPos_Kp_x = 0.
      if self.cartPos_Kp_y is None:
        self.cartPos_Kp_y = 0.
      if self.cartPos_Kp_z is None:
        self.cartPos_Kp_z = 0.
      if self.cartPos_Kd_x is None:
        self.cartPos_Kd_x = 0.
      if self.cartPos_Kd_y is None:
        self.cartPos_Kd_y = 0.
      if self.cartPos_Kd_z is None:
        self.cartPos_Kd_z = 0.
      if self.cartRot_Kp_x is None:
        self.cartRot_Kp_x = 0.
      if self.cartRot_Kp_y is None:
        self.cartRot_Kp_y = 0.
      if self.cartRot_Kp_z is None:
        self.cartRot_Kp_z = 0.
      if self.cartRot_Kd_x is None:
        self.cartRot_Kd_x = 0.
      if self.cartRot_Kd_y is None:
        self.cartRot_Kd_y = 0.
      if self.cartRot_Kd_z is None:
        self.cartRot_Kd_z = 0.
      if self.useCurrentCartPose is None:
        self.useCurrentCartPose = False
      if self.useNullspacePose is None:
        self.useNullspacePose = False
      if self.cartIniX is None:
        self.cartIniX = 0.
      if self.cartIniY is None:
        self.cartIniY = 0.
      if self.cartIniZ is None:
        self.cartIniZ = 0.
      if self.cartIniRoll is None:
        self.cartIniRoll = 0.
      if self.cartIniPitch is None:
        self.cartIniPitch = 0.
      if self.cartIniYaw is None:
        self.cartIniYaw = 0.
      if self.cartDesX is None:
        self.cartDesX = 0.
      if self.cartDesY is None:
        self.cartDesY = 0.
      if self.cartDesZ is None:
        self.cartDesZ = 0.
      if self.cartDesRoll is None:
        self.cartDesRoll = 0.
      if self.cartDesPitch is None:
        self.cartDesPitch = 0.
      if self.cartDesYaw is None:
        self.cartDesYaw = 0.
      if self.m is None:
        self.m = 0.
      if self.d is None:
        self.d = 0.
      if self.k is None:
        self.k = 0.
      if self.task_mA is None:
        self.task_mA = 0.
      if self.task_mB is None:
        self.task_mB = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.kappa = 0.
      self.Kv = 0.
      self.lambda_ = 0.
      self.Kz = 0.
      self.Zb = 0.
      self.F = 0.
      self.G = 0.
      self.inParams = 0
      self.outParams = 0
      self.hiddenNodes = 0
      self.errorParams = 0
      self.feedForwardForce = 0.
      self.nn_ON = 0.
      self.cartPos_Kp_x = 0.
      self.cartPos_Kp_y = 0.
      self.cartPos_Kp_z = 0.
      self.cartPos_Kd_x = 0.
      self.cartPos_Kd_y = 0.
      self.cartPos_Kd_z = 0.
      self.cartRot_Kp_x = 0.
      self.cartRot_Kp_y = 0.
      self.cartRot_Kp_z = 0.
      self.cartRot_Kd_x = 0.
      self.cartRot_Kd_y = 0.
      self.cartRot_Kd_z = 0.
      self.useCurrentCartPose = False
      self.useNullspacePose = False
      self.cartIniX = 0.
      self.cartIniY = 0.
      self.cartIniZ = 0.
      self.cartIniRoll = 0.
      self.cartIniPitch = 0.
      self.cartIniYaw = 0.
      self.cartDesX = 0.
      self.cartDesY = 0.
      self.cartDesZ = 0.
      self.cartDesRoll = 0.
      self.cartDesPitch = 0.
      self.cartDesYaw = 0.
      self.m = 0.
      self.d = 0.
      self.k = 0.
      self.task_mA = 0.
      self.task_mB = 0.

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d4q14d2B17d.pack(_x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 338
      (_x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB,) = _struct_7d4q14d2B17d.unpack(str[start:end])
      self.useCurrentCartPose = bool(self.useCurrentCartPose)
      self.useNullspacePose = bool(self.useNullspacePose)
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d4q14d2B17d.pack(_x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 338
      (_x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB,) = _struct_7d4q14d2B17d.unpack(str[start:end])
      self.useCurrentCartPose = bool(self.useCurrentCartPose)
      self.useNullspacePose = bool(self.useNullspacePose)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_7d4q14d2B17d = struct.Struct("<7d4q14d2B17d")
