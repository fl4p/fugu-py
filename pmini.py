"""Compatibility import for the revision-pinned standalone MTP2 codec.

The implementation is vendored byte-for-byte; see pmini.lock.json and PMINI-LICENSE.
PMA2 and telemetry schema logic remain in teledec.py.
"""
try:
    from ._pmini_mtp2 import MAGIC, MAX_ROWS, inspect, decode_words, decode_rows
except ImportError:
    from _pmini_mtp2 import MAGIC, MAX_ROWS, inspect, decode_words, decode_rows
