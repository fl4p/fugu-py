"""Decoder for the firmware's binary telemetry wire (sym_line_protocol.h).

Shared by etc/influx_binary_proxy.py (UDP datagrams / BLE records) and
fugu_console.py --tele. Payload format:

  payload  = (<varint len> <frame>)*  ; frame[0] = FrameT (1=data, 2=table)
  table : (<SID varint> <name\\0> <DT byte>)*  <0-SID>
  data  : <SID(meas)> (<SID tagK><SID tagV>)* 0
                       (<SID fieldK><raw LE value>)* 0  <ts_ms varint>

Over UDP one datagram = <cid:1B><payload>. Over BLE (a byte stream) records are
framed <0x7E><varint len><cid><payload>; TeleStream reassembles and resyncs.
"""
import struct

try:
    import tamp
except ImportError:
    tamp = None

# WireDT -> (struct fmt, width). Str(0) has no value (appears only as a SID).
DT = {1: ('?', 1), 2: ('<b', 1), 3: ('<B', 1), 4: ('<h', 2), 5: ('<H', 2),
      6: ('<i', 4), 7: ('<I', 4), 8: ('<e', 2), 9: ('<f', 4), 10: ('<d', 8)}
DT_STR = 0

RECORD_MAGIC = 0x7E
MAX_RECORD = 4096   # device raw cap is 2 KB (tamp worst case ~2.4 KB); tight bound aids resync


class Reader:
    def __init__(self, b): self.b, self.i = b, 0
    def varint(self):
        v, s = 0, 0
        while True:
            x = self.b[self.i]; self.i += 1
            v |= (x & 0x7F) << s
            if not (x & 0x80): return v
            s += 7
    def cstr(self):
        j = self.b.index(0, self.i); s = self.b[self.i:j].decode(); self.i = j + 1; return s
    def byte(self):
        x = self.b[self.i]; self.i += 1; return x
    def take(self, n):
        x = self.b[self.i:self.i + n]; self.i += n; return x
    def eof(self): return self.i >= len(self.b)


def fmt_field(dt, raw):
    fmt, _ = DT[dt]
    v = struct.unpack(fmt, raw)[0]
    if dt == 1:  return 'true' if v else 'false'         # bool
    if dt <= 7:  return f'{v}i'                            # int -> influx integer
    return f'{v:.6g}'                                      # float


def decode_payload(cid, payload, tab):
    """Yield influx line-protocol strings from one <cid><payload> unit. tab (SID->(name,dt)) persists."""
    if cid == 1:
        if tamp is None:
            raise RuntimeError("payload is tamp-compressed but 'tamp' is not installed (pip install tamp)")
        payload = tamp.decompress(bytes(payload))
    elif cid != 0:
        raise ValueError(f"unknown compressor id {cid}")
    r = Reader(payload)
    while not r.eof():
        flen = r.varint()
        frame = Reader(r.take(flen))
        ft = frame.byte()
        if ft == 2:                          # table
            while True:
                sid = frame.varint()
                if sid == 0: break
                tab[sid] = (frame.cstr(), frame.byte())
        elif ft == 1:                        # data
            def name(sid): return tab[sid][0]
            meas = name(frame.varint())
            tags = []
            while frame.b[frame.i] != 0:
                k = name(frame.varint()); v = name(frame.varint()); tags.append(f'{k}={v}')
            frame.i += 1                     # tag terminator
            fields = []
            while frame.b[frame.i] != 0:
                sid = frame.varint(); _, dt = tab[sid]
                fields.append(f'{name(sid)}={fmt_field(dt, frame.take(DT[dt][1]))}')
            frame.i += 1                     # field terminator
            ts = frame.varint()
            line = meas + (',' + ','.join(tags) if tags else '') + ' ' + ','.join(fields) + ' ' + str(ts)
            yield line
        else:
            raise ValueError(f"unknown frame type {ft}")


class TeleStream:
    """Reassemble <0x7E><varint len><cid><payload> records from a notify byte stream.

    feed() returns decoded influx lines. 0x7E occurs freely inside tamp payloads, so
    resync validates the length bound + compressor id and, on any parse/decode failure,
    rescans from the next 0x7E.
    """

    def __init__(self, on_error=None):
        self.buf = bytearray()
        self.tab = {}
        self.on_error = on_error

    def _resync(self):
        nxt = self.buf.find(RECORD_MAGIC, 1)
        del self.buf[:nxt if nxt > 0 else len(self.buf)]

    def feed(self, data: bytes):
        self.buf += data
        lines = []
        while self.buf:
            if self.buf[0] != RECORD_MAGIC:
                self._resync(); continue
            # varint length after the magic; incomplete -> wait for more bytes
            n, s, i = 0, 0, 1
            while True:
                if i >= len(self.buf): return lines
                x = self.buf[i]; i += 1
                n |= (x & 0x7F) << s; s += 7
                if not (x & 0x80): break
                if s > 28: n = MAX_RECORD + 1; break
            if not 1 <= n <= MAX_RECORD or (len(self.buf) > i and self.buf[i] not in (0, 1)):
                self._resync(); continue
            if len(self.buf) < i + n:
                return lines                    # record incomplete
            rec = bytes(self.buf[i:i + n])
            # The record is complete and its framing validated: on a semantic decode error
            # (e.g. unknown SID after a dropped table frame) skip THIS record and keep the
            # framing — a byte-wise resync would land mid-payload and cascade.
            del self.buf[:i + n]
            try:
                lines += decode_payload(rec[0], rec[1:], self.tab)
            except Exception as e:
                if self.on_error: self.on_error(e)
        return lines
