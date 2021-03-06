"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class acState_t(object):
    __slots__ = ["timestamp", "aircraftID", "position", "velocity", "orientation", "nextRouteID"]

    def __init__(self):
        self.timestamp = 0
        self.aircraftID = 0.0
        self.position = [ 0.0 for dim0 in range(3) ]
        self.velocity = [ 0.0 for dim0 in range(3) ]
        self.orientation = [ 0.0 for dim0 in range(3) ]
        self.nextRouteID = 0

    def encode(self):
        buf = BytesIO()
        buf.write(acState_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qd", self.timestamp, self.aircraftID))
        buf.write(struct.pack('>3d', *self.position[:3]))
        buf.write(struct.pack('>3d', *self.velocity[:3]))
        buf.write(struct.pack('>3d', *self.orientation[:3]))
        buf.write(struct.pack(">q", self.nextRouteID))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != acState_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return acState_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = acState_t()
        self.timestamp, self.aircraftID = struct.unpack(">qd", buf.read(16))
        self.position = struct.unpack('>3d', buf.read(24))
        self.velocity = struct.unpack('>3d', buf.read(24))
        self.orientation = struct.unpack('>3d', buf.read(24))
        self.nextRouteID = struct.unpack(">q", buf.read(8))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if acState_t in parents: return 0
        tmphash = (0xbb1f210406a712ab) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if acState_t._packed_fingerprint is None:
            acState_t._packed_fingerprint = struct.pack(">Q", acState_t._get_hash_recursive([]))
        return acState_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

