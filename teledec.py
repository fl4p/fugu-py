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
from binascii import crc32

try:
    from . import pmini
except ImportError:                         # script-style imports
    import pmini

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
PMA2_MAGIC = b'PMA2'
PMA2_MAX_EVENTS = 512
PMA2_MAX_PREFIX = 4096
PMA2_MAX_UNION = 32
PMA2_MAX_ROWS = 128
PMA2_MAX_SID = 4096
PMA2_MAX_PACKET = 8192


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


def _read_varint_strict(data, pos, end):
    value = 0
    shift = 0
    while True:
        if pos >= end:
            raise ValueError('PMA2 truncated varint')
        x = data[pos]; pos += 1
        value |= (x & 0x7f) << shift
        if not (x & 0x80):
            if shift and x == 0:
                raise ValueError('PMA2 non-canonical varint')
            return value, pos
        shift += 7
        if shift > 35:
            raise ValueError('PMA2 varint too long')


def decode_packet(payload):
    """Decode a cid=2 PMA2 packet into the original length-prefixed frames.

    PMA2 deliberately reconstructs bytes before invoking the established wire
    decoder.  All checks happen into local objects, so malformed packets cannot
    partially update the caller's symbol table.
    """
    data = bytes(payload)
    if len(data) < 4 + 1 + 4 or len(data) > PMA2_MAX_PACKET or data[:4] != PMA2_MAGIC:
        raise ValueError('PMA2 bad magic or truncated envelope')
    if crc32(data[:-4]) & 0xffffffff != struct.unpack_from('<I', data, len(data) - 4)[0]:
        raise ValueError('PMA2 CRC-32 mismatch')
    end = len(data) - 4
    pos = 4
    version = data[pos]; pos += 1
    if version != 1:
        raise ValueError(f'PMA2 unsupported version {version}')
    events, pos = _read_varint_strict(data, pos, end)
    if not 1 <= events <= PMA2_MAX_EVENTS:
        raise ValueError('PMA2 invalid event count')
    prefix_len, pos = _read_varint_strict(data, pos, end)
    if not 1 <= prefix_len <= PMA2_MAX_PREFIX or prefix_len > end - pos:
        raise ValueError('PMA2 invalid common prefix length')
    prefix = data[pos:pos + prefix_len]; pos += prefix_len
    # The common prefix is exactly a data-frame prefix: frame type, measurement
    # SID, tag pairs, and the tag terminator.  Parse it to reject ambiguity.
    pr = Reader(prefix)
    if pr.byte() != 1:
        raise ValueError('PMA2 common prefix is not a data frame')
    pr.varint()
    while True:
        sid = pr.varint()
        if sid == 0:
            break
        if pr.varint() == 0:
            raise ValueError('PMA2 tag value SID is zero')
    if not pr.eof():
        raise ValueError('PMA2 common prefix has trailing bytes')
    union_count, pos = _read_varint_strict(data, pos, end)
    if not 1 <= union_count <= PMA2_MAX_UNION:
        raise ValueError('PMA2 invalid union field count')
    union = []
    seen = set()
    for _ in range(union_count):
        sid, pos = _read_varint_strict(data, pos, end)
        if sid == 0 or sid > PMA2_MAX_SID or sid in seen:
            raise ValueError('PMA2 duplicate or zero union SID')
        if pos >= end:
            raise ValueError('PMA2 truncated union datatype')
        dt = data[pos]; pos += 1
        if dt not in DT or dt == DT_STR or DT[dt][1] > 4:
            raise ValueError('PMA2 unsupported union datatype')
        seen.add(sid); union.append((sid, dt))

    descriptors = []
    table_literals = []
    for _ in range(events):
        if pos >= end:
            raise ValueError('PMA2 truncated event descriptors')
        kind = data[pos]; pos += 1
        if kind == 0:
            raw_len, pos = _read_varint_strict(data, pos, end)
            if not 1 <= raw_len <= MAX_RECORD or raw_len > end - pos:
                raise ValueError('PMA2 invalid table literal length')
            literal = data[pos:pos + raw_len]; pos += raw_len
            # Exact original bytes are retained, but validate its outer length
            # prefix and table frame type before accepting it.
            flen, fp = _read_varint_strict(literal, 0, len(literal))
            if fp + flen != len(literal) or flen < 1 or literal[fp] != 2:
                raise ValueError('PMA2 invalid table literal')
            table_literals.append(literal)
            descriptors.append(literal)
        elif kind == 1:
            descriptors.append(None)
        else:
            raise ValueError('PMA2 unknown event descriptor')
    shard = data[pos:end]
    if not shard:
        raise ValueError('PMA2 missing MTP2 shard')
    # The first three columns are timestamp low/high and a 32-bit presence mask.
    cols = [('__tslo', 7), ('__tshi', 7), ('__present', 7)] + [(str(sid), dt) for sid, dt in union]
    data_count = sum(item is None for item in descriptors)
    shard_rows, shard_cols = pmini.inspect(shard)
    if not 1 <= data_count <= PMA2_MAX_ROWS or shard_rows != data_count or shard_cols != len(cols):
        raise ValueError('PMA2 row/event or column count mismatch')
    words = pmini.decode_rows(shard, cols)
    if len(words) != data_count:
        raise ValueError('PMA2 row/event count mismatch')
    # Rebuild the original frame stream. Every data row is represented by one
    # descriptor, while table literals retain their exact original prefix.
    raw = bytearray()
    row_index = 0
    for item in descriptors:
        if item is not None:
            raw += item
            continue
        row = words[row_index]; row_index += 1
        lo, hi, present = row[:3]
        valid_mask = (1 << len(union)) - 1
        if present & ~valid_mask:
            raise ValueError('PMA2 presence mask has unknown field bits')
        if any(row[3 + bit] for bit in range(len(union)) if not present & (1 << bit)):
            raise ValueError('PMA2 absent field word is nonzero')
        timestamp = lo | (hi << 32)
        frame = bytearray(prefix)
        for bit, (sid, dt) in enumerate(union):
            if present & (1 << bit):
                width = DT[dt][1]
                if row[3 + bit] >> (8 * width):
                    raise ValueError('PMA2 field word overflows its datatype')
                frame += _encode_varint(sid)
                frame += struct.pack('<I', row[3 + bit])[:width]
        frame.append(0)
        frame += _encode_varint(timestamp)
        raw += _encode_varint(len(frame)) + frame
    return bytes(raw)


# Public name used by host/native cross-language replay tests.
_pmini_data_frames = decode_packet


def _encode_varint(value):
    if value < 0:
        raise ValueError('negative varint')
    out = bytearray()
    while value >= 0x80:
        out.append((value & 0x7f) | 0x80); value >>= 7
    out.append(value)
    return bytes(out)


def decode_payload(cid, payload, tab):
    """Yield influx line-protocol strings from one <cid><payload> unit. tab (SID->(name,dt)) persists."""
    if cid == 2:
        # Decode and apply against a copy: neither a bad PMA2 envelope nor a
        # malformed reconstructed table frame may partially mutate `tab`.
        local = dict(tab)
        lines = list(decode_payload(0, _pmini_data_frames(payload), local))
        tab.clear(); tab.update(local)
        yield from lines
        return
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
            try:
                meas = name(frame.varint())
            except KeyError:                 # table not seen yet: skip this frame, keep going
                continue
            tags = []
            try:
                while frame.b[frame.i] != 0:
                    k = name(frame.varint()); v = name(frame.varint()); tags.append(f'{k}={v}')
            except KeyError:
                continue
            frame.i += 1                     # tag terminator
            fields = []
            try:
                while frame.b[frame.i] != 0:
                    sid = frame.varint(); _, dt = tab[sid]
                    fields.append(f'{name(sid)}={fmt_field(dt, frame.take(DT[dt][1]))}')
            except KeyError:
                continue
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
            bad_cid2_header = (len(self.buf) > i and self.buf[i] == 2 and
                               (n < 5 or (len(self.buf) >= i + n and bytes(self.buf[i + 1:i + 5]) != PMA2_MAGIC)))
            if not 1 <= n <= MAX_RECORD or (len(self.buf) > i and self.buf[i] not in (0, 1, 2)) or bad_cid2_header:
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
