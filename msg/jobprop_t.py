"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class jobprop_t(object):
    __slots__ = ["timestamp", "aircraftID", "intersectionID", "release", "deadline"]

    def __init__(self):
        self.timestamp = 0
        self.aircraftID = 0
        self.intersectionID = 0
        self.release = 0.0
        self.deadline = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(jobprop_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qqqdd", self.timestamp, self.aircraftID, self.intersectionID, self.release, self.deadline))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != jobprop_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return jobprop_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = jobprop_t()
        self.timestamp, self.aircraftID, self.intersectionID, self.release, self.deadline = struct.unpack(">qqqdd", buf.read(40))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if jobprop_t in parents: return 0
        tmphash = (0x35dfa6b92cceddb9) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if jobprop_t._packed_fingerprint is None:
            jobprop_t._packed_fingerprint = struct.pack(">Q", jobprop_t._get_hash_recursive([]))
        return jobprop_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

