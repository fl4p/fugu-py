"""Small, strict host-side decoder for Fabian Schlieper's pmini MTP2 format.

This is the decoder subset of ``pmini/pmini.py``.  It intentionally has no
encoder, workspace, or MicroPython dependency: codec-2 packets are untrusted
network input and all dimensions/streams are checked before values are
materialized.

Derived from the decoder in pmini/pmini.py, Copyright (c) 2026 Fabian
Schlieper, under the MIT License: permission is granted, free of charge, to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of this software, subject to inclusion of this notice and the copyright
notice.  The software is provided "AS IS", without warranty.
"""
from binascii import crc32
import struct

MAGIC = b"MTP2"
MAX_ROWS = 4096
M32 = 0xffffffff
_CONST, _RAW, _D1, _DICT, _SPLIT, _SAME = 0, 1, 2, 4, 5, 6
_NB = 136


class _Bits:
    def __init__(self, data, start, end):
        self.data, self.p, self.end = data, start, end
        self.acc = 0
        self.n = 0

    def read(self, n):
        if not 0 <= n <= 32:
            raise ValueError("pmini: invalid bit count")
        while self.n < n:
            if self.p >= self.end:
                raise ValueError("pmini: bit stream overrun")
            self.acc |= self.data[self.p] << self.n
            self.p += 1
            self.n += 8
        value = self.acc & ((1 << n) - 1) if n else 0
        self.acc >>= n
        self.n -= n
        return value

    def gamma(self):
        zeros = 0
        while self.read(1) == 0:
            zeros += 1
            if zeros > 32:
                raise ValueError("pmini: bad gamma code")
        return ((1 << zeros) | self.read(zeros)) - 1

    def big(self):
        n = self.read(6)
        if n > 32:
            raise ValueError("pmini: value wider than 32 bits")
        return self.read(n)

    def finish(self):
        if self.p != self.end or self.acc:
            raise ValueError("pmini: bit stream not consumed to padding")


class _Rans:
    def __init__(self, data, start, end):
        self.data, self.start, self.p = data, start, end
        self.x = None
        if end > start:
            if end - start < 3:
                raise ValueError("pmini: rANS stream too short")
            self.x = data[end - 1] << 16 | data[end - 2] << 8 | data[end - 3]
            self.p = end - 3

    def finish(self):
        if self.x is not None and (self.x != 1 << 16 or self.p != self.start):
            raise ValueError("pmini: rANS stream did not return to its initial state")


def _unbucket(symbol, k, bits):
    if symbol < (1 << k):
        return symbol
    t = symbol - (1 << k)
    exponent = t >> k
    return (((1 << k) + (t & ((1 << k) - 1))) << exponent) | bits.read(exponent)


def _latents(bits, rans, count):
    if count > MAX_ROWS:
        raise ValueError("pmini: latent stream exceeds MAX_ROWS")
    k = bits.read(2)
    n_used = bits.gamma() + 1
    if n_used > _NB or n_used > count:
        raise ValueError("pmini: invalid bucket count")
    max_symbol = (1 << k) + ((31 - k) << k) + (1 << k) - 1
    used, previous = [], -1
    for _ in range(n_used):
        previous += 1 + bits.gamma()
        if previous > max_symbol:
            raise ValueError("pmini: bucket id out of range")
        used.append(previous)
    counts, remaining = [], count
    for _ in range(n_used - 1):
        c = bits.gamma() + 1
        counts.append(c)
        remaining -= c
    if remaining < 1:
        raise ValueError("pmini: frequency table does not sum to stream length")
    counts.append(remaining)
    freqs = [max(1, (c << 12) // count) for c in counts]
    largest = max(range(n_used), key=freqs.__getitem__)
    freqs[largest] += 4096 - sum(freqs)
    if freqs[largest] <= 0:
        raise ValueError("pmini: invalid frequency table")
    lut = bytearray(4096)
    cumulative, at = [], 0
    for index, frequency in enumerate(freqs):
        cumulative.append(at)
        lut[at:at + frequency] = bytes([index]) * frequency
        at += frequency
    if rans.x is None:
        raise ValueError("pmini: latent stream without rANS stream")
    x, position = rans.x, rans.p
    out = []
    for _ in range(count):
        slot = x & 4095
        index = lut[slot]
        x = freqs[index] * (x >> 12) + slot - cumulative[index]
        while x < (1 << 16):
            position -= 1
            if position < rans.start:
                raise ValueError("pmini: rANS stream overrun")
            x = (x << 8) | rans.data[position]
        out.append(_unbucket(used[index], k, bits))
    rans.x, rans.p = x, position
    return out


def _unzigzag(value):
    return value >> 1 if not value & 1 else -((value + 1) >> 1)


def _simple(bits, rans, transform, count):
    if transform == _RAW:
        minimum = bits.big()
        return [(value + minimum) & M32 for value in _latents(bits, rans, count)]
    if transform == _D1:
        value = bits.big()
        result = [value]
        for delta in _latents(bits, rans, count - 1) if count > 1 else ():
            value = (value + _unzigzag(delta)) & M32
            result.append(value)
        return result
    if transform == _DICT:
        n_values = bits.gamma() + 1
        if n_values > 32:
            raise ValueError("pmini: dictionary has too many values")
        dictionary = [bits.big()]
        for _ in range(n_values - 1):
            dictionary.append(dictionary[-1] + 1 + bits.gamma())
        result = []
        for index in _latents(bits, rans, count):
            if index >= n_values:
                raise ValueError("pmini: dictionary index out of range")
            result.append(dictionary[index])
        return result
    raise ValueError("pmini: bad transform tag")


def _read_varint(data, position, limit):
    value = 0
    shift = 0
    while True:
        if position >= limit:
            raise ValueError("pmini: truncated varint")
        byte = data[position]
        position += 1
        value |= (byte & 0x7f) << shift
        if not byte & 0x80:
            return value, position
        shift += 7
        if shift > 35:
            raise ValueError("pmini: varint too long")


def inspect(blob):
    """Validate the fixed MTP2 header and return ``(rows, columns)``.

    This deliberately does not allocate or decode blocks.  PMA2 uses it to
    reject its tighter 128-row envelope bound before doing any rANS work.
    """
    data = bytes(blob)
    if len(data) < 10 or data[:4] != MAGIC:
        raise ValueError("not a pmini MTP2 shard")
    if crc32(data[:-4]) & M32 != struct.unpack_from("<I", data, len(data) - 4)[0]:
        raise ValueError("pmini: CRC-32 mismatch")
    end = len(data) - 4
    rows, position = _read_varint(data, 4, end)
    columns, position = _read_varint(data, position, end)
    if rows > MAX_ROWS:
        raise ValueError("pmini: rows exceed MAX_ROWS")
    return rows, columns


def decode_words(blob, columns):
    """Return MTP2 row-major columns as unsigned 32-bit words.

    ``columns`` is only used for its length.  The envelope carries field
    datatypes separately, so this function deliberately does not reinterpret
    signed/floating values.
    """
    data = bytes(blob)
    rows, ncols = inspect(data)
    end = len(data) - 4
    position = 4
    _, position = _read_varint(data, position, end)
    _, position = _read_varint(data, position, end)
    if ncols != len(columns):
        raise ValueError("pmini: shard column count does not match schema")
    blocks = []
    trailer_end = end
    while trailer_end > position:
        if trailer_end - 6 < position:
            raise ValueError("pmini: truncated block trailer")
        bit_len, rans_len, block_cols = struct.unpack_from("<HHH", data, trailer_end - 6)
        start = trailer_end - 6 - bit_len - rans_len
        if start < position or not block_cols:
            raise ValueError("pmini: bad block trailer")
        blocks.append((start, bit_len, rans_len, block_cols))
        trailer_end = start
    blocks.reverse()
    if rows == 0:
        if blocks:
            raise ValueError("pmini: blocks in empty shard")
        return [[] for _ in columns]
    if sum(block[3] for block in blocks) != ncols:
        raise ValueError("pmini: block columns do not match header")
    result, gap_streams = [], {}
    for start, bit_len, rans_len, block_cols in blocks:
        bits = _Bits(data, start, start + bit_len)
        rans = _Rans(data, start + bit_len, start + bit_len + rans_len)
        for _ in range(block_cols):
            column = len(result)
            transform = bits.read(3)
            if transform == _CONST:
                result.append([bits.big()] * rows)
                continue
            if transform in (_SPLIT, _SAME):
                background = bits.big()
                if transform == _SPLIT:
                    n_foreground = bits.gamma() + 1
                    if n_foreground >= rows:
                        raise ValueError("pmini: invalid SPLIT count")
                    gaps = _latents(bits, rans, n_foreground)
                    gap_streams[column] = gaps
                else:
                    reference = column - bits.gamma() - 1
                    if reference not in gap_streams:
                        raise ValueError("pmini: SAME references non-SPLIT column")
                    gaps = gap_streams[reference]
                foreground = _simple(bits, rans, bits.read(3), len(gaps))
                values = [background] * rows
                row = -1
                for gap, value in zip(gaps, foreground):
                    row += gap + 1
                    if row >= rows:
                        raise ValueError("pmini: SPLIT position out of range")
                    values[row] = value
                result.append(values)
            else:
                result.append(_simple(bits, rans, transform, rows))
        bits.finish()
        rans.finish()
    return result


def decode_rows(blob, columns):
    """Decode an MTP2 shard to rows of raw u32 words."""
    cols = decode_words(blob, columns)
    return [tuple(col[row] for col in cols) for row in range(len(cols[0]) if cols else 0)]
