#!/usr/bin/env python3
"""bin2tap.py -- pack a raw Z80 binary into a ZX Spectrum .tap tape image.

The generated tape can be loaded in any emulator (Fuse, ZEsarUX, Spectaculator,
UnrealSpeccy, ...) or written to a real cassette / played as audio.

By default two things are written:

  1. a one-line BASIC program with an autostart line, so the emulator runs the
     code as soon as the tape is loaded:
         10 CLEAR <org-1>: LOAD ""CODE: RANDOMIZE USR <org>
  2. a CODE block holding the binary itself, with its start address in the
     tape header, which is what bare `LOAD ""CODE` picks up.

An optional 48K .sna snapshot can be written as well.  That skips the tape
loading entirely: the emulator starts with the code already in memory, so the
terminal is running the moment the snapshot is opened.

Usage
-----
    python bin2tap.py terminal.bin                  -> terminal.tap
    python bin2tap.py terminal.bin --sna            -> + terminal.sna
    python bin2tap.py code.bin -o game.tap --org 32768 --name TERMINAL
    python bin2tap.py code.bin --no-basic           -> code block only
    python bin2tap.py code.bin --no-autorun         -> loader without autostart

Only Python 3 standard library is required.
"""

import argparse
import os
import struct
import sys

# --------------------------------------------------------------------------
# ZX Spectrum tape format
# --------------------------------------------------------------------------
# A .tap file is a flat concatenation of blocks.  Every block is
#      <2 byte little-endian length> <flag> <length bytes of data> <checksum>
# where the checksum is the XOR of the flag byte and every data byte.
#
# A 17 byte header block (flag = 0x00 when loading, but stored with the same
# framing) is:
#      <type> <10 char name> <data length> <param1> <param2>
# and is followed by a data block (flag = 0xFF) holding the payload.

HEADER_FLAG = 0x00
DATA_FLAG = 0xFF

TYPE_PROGRAM = 0
TYPE_NUMARRAY = 1
TYPE_CHRARRAY = 2
TYPE_BYTES = 3

ROM_NO_AUTOSTART = 0x8000  # 32768 means "do not run automatically"


def checksum(flag, data):
    x = flag
    for b in data:
        x ^= b
    return x


def tap_block(data):
    """Frame `data` (flag + payload) as a length-prefixed tape block."""
    block = bytes(data)
    block += bytes([checksum(flag_of(block), block[1:])])
    return struct.pack("<H", len(block)) + block


def flag_of(block):
    return block[0]


def header_block(block_type, name, length, param1, param2):
    name = name.encode("ascii", "replace")[:10].ljust(10, b" ")
    payload = (bytes([block_type]) + name + struct.pack("<HHH", length, param1, param2))
    assert len(payload) == 17
    return tap_block(bytes([HEADER_FLAG]) + payload)


def data_block(data):
    return tap_block(bytes([DATA_FLAG]) + bytes(data))


# --------------------------------------------------------------------------
# ZX Spectrum BASIC tokenising (only what the loader line needs)
# --------------------------------------------------------------------------
TOKENS = {
    "CODE": 0xAF,
    "CLEAR": 0xFD,
    "LOAD": 0xEF,
    "RANDOMIZE": 0xF9,
    "USR": 0xC0,
}


def zx_number(n):
    """5 byte ZX Spectrum floating point form of a non-negative integer.

    Format: <exponent + 128> <32 bit mantissa, big endian>.  The value is
        2 ** (exponent - 128) * (0.5 + mantissa / 2**32)
    i.e. the leading 0.5 of the mantissa is implied, so all-zero mantissa
    bytes mean exactly a power of two and the top mantissa bit is the sign
    (always 0 here).  That gives the familiar 1.0 = 81 00 00 00 00 and
    32768 = 90 00 00 00 00.
    """
    if n < 0:
        raise ValueError("negative numbers are not supported")
    if n == 0:
        return bytes(5)
    exponent = n.bit_length()               # n / 2**exponent in [0.5, 1)
    mantissa = (n << (32 - exponent)) - 0x80000000
    return bytes([(exponent + 128) & 0xFF,
                  (mantissa >> 24) & 0xFF,
                  (mantissa >> 16) & 0xFF,
                  (mantissa >> 8) & 0xFF,
                  mantissa & 0xFF])


def zx_num_literal(n):
    """A numeric literal as stored inside a BASIC line: ASCII digits, the
    0x0E marker, then the five byte binary value."""
    return str(n).encode("ascii") + bytes([0x0E]) + zx_number(n)


def build_loader(clear, start, autostart=True):
    """Return the tokenised BASIC program bytes for the loader line."""
    body = b""
    body += bytes([TOKENS["CLEAR"], 0x20]) + zx_num_literal(clear)
    body += b":"
    body += bytes([TOKENS["LOAD"], 0x20]) + b'""' + bytes([TOKENS["CODE"]])
    body += b":" + bytes([TOKENS["RANDOMIZE"], 0x20, TOKENS["USR"], 0x20])
    body += zx_num_literal(start)

    line_no = 10
    line = struct.pack(">HH", line_no, len(body) + 1) + body + b"\x0d"
    program = line + b"\x0d"          # program ends with a lone 0x0D
    return program, (line_no if autostart else ROM_NO_AUTOSTART)


# --------------------------------------------------------------------------
# Listing the program back out (used by --list and by the test suite)
# --------------------------------------------------------------------------
def list_basic(program):
    """Decode tokenised BASIC back into text.  Round trip evidence that the
    loader line really is what we think it is."""
    text = []
    i = 0
    while i < len(program):
        if program[i] == 0x0D:
            i += 1
            continue
        if i + 4 > len(program):
            break
        number, length = struct.unpack(">HH", program[i:i + 4])
        body = program[i + 4:i + 4 + length - 1]
        i += 4 + length
        out = []
        j = 0
        while j < len(body):
            b = body[j]
            if b == 0x0E:
                # The ASCII digits we just emitted are followed by the binary
                # value.  Replace the digits with the decoded number so the
                # listing also proves the encoding round trips.
                digits = 0
                while digits < len(out) and out[-1 - digits].isdigit():
                    digits += 1
                chunk = body[j + 1:j + 6]
                if len(chunk) == 5:
                    del out[len(out) - digits:]
                    out.append(str(decode_zx_number(chunk)))
                j += 6
                continue
            if b in _REVERSE_TOKENS:
                out.append(_REVERSE_TOKENS[b])
            elif 0x20 <= b < 0x7F:
                out.append(chr(b))
            else:
                out.append("{0x%02X}" % b)
            j += 1
        text.append("%d %s" % (number, "".join(out)))
    return "\n".join(text)


_REVERSE_TOKENS = {v: k for k, v in TOKENS.items()}


def decode_zx_number(b):
    """Inverse of zx_number()."""
    if b == bytes(5):
        return 0
    exponent = b[0] - 128
    mantissa = (b[1] << 24) | (b[2] << 16) | (b[3] << 8) | b[4]
    if exponent == 0:
        return 0
    value = (0.5 + mantissa / 4294967296.0) * (2.0 ** exponent)
    return int(round(value))


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def make_tap(code, org, name, want_basic=True, autostart=True):
    if not 0x4000 <= org < 0x10000:
        raise ValueError("org 0x%04X is not loadable RAM (0x4000..0xFFFF)" % org)
    if org + len(code) > 0x10000:
        raise ValueError("code starting at 0x%04X runs past the top of the "
                         "address space" % org)
    out = b""
    if want_basic:
        program, start_line = build_loader(org - 1, org, autostart)
        out += header_block(TYPE_PROGRAM, name, len(program),
                            start_line, len(program))
        out += data_block(program)
    out += header_block(TYPE_BYTES, name, len(code), org, 32768)
    out += data_block(code)
    return out


# --------------------------------------------------------------------------
# 48K .sna snapshot
# --------------------------------------------------------------------------
# Layout: a 27 byte register header followed by a straight dump of RAM from
# 0x4000 to 0xFFFF (49152 bytes), so the file is 49179 bytes.  There is no
# ROM image in a .sna; the emulator supplies its own.
SNA_HEADER_SIZE = 27
SNA_RAM_START = 0x4000
SNA_RAM_SIZE = 0xC000

SNA_I = 0x3F            # what the 48K ROM leaves in I
SNA_R = 0x00
SNA_IM = 1              # ROM default; irrelevant to the terminal, which DI's
SNA_IFF2 = 0            # bit 2 clear -> interrupts disabled


def make_sna(code, org, border=0):
    """Build a 48K snapshot that starts executing `code` at `org`.

    A .sna has no PC field: the emulator loads the registers and then does an
    implicit RET, so the entry point has to be sitting on the stack.  Exiting
    the terminal does JP 0, which is a full ROM restart, so the blank system
    variables this snapshot ships with are harmless.
    """
    if not (SNA_RAM_START <= org < SNA_RAM_START + SNA_RAM_SIZE):
        raise ValueError("org 0x%04X is outside 48K RAM" % org)
    if org + len(code) > SNA_RAM_START + SNA_RAM_SIZE:
        raise ValueError("code runs past the top of RAM")

    # Initial stack just below the code, i.e. where the program itself puts it.
    sp = org - 2
    if sp < SNA_RAM_START + 2:
        sp = SNA_RAM_START + SNA_RAM_SIZE - 2          # 0xFFFE
    if org <= sp < org + len(code):
        raise ValueError("no room for the initial stack")

    ram = bytearray(SNA_RAM_SIZE)
    ram[org - SNA_RAM_START:org - SNA_RAM_START + len(code)] = code
    ram[sp - SNA_RAM_START:sp - SNA_RAM_START + 2] = struct.pack("<H", org)

    header = bytearray(SNA_HEADER_SIZE)
    header[0] = SNA_I
    # HL', DE', BC', AF' and HL, DE, BC, IY, IX stay zero
    header[19] = SNA_IFF2
    header[20] = SNA_R
    header[21:23] = struct.pack("<H", 0x0000)          # AF
    header[23:25] = struct.pack("<H", sp)              # SP
    header[25] = SNA_IM
    header[26] = border & 0x07
    return bytes(header) + bytes(ram)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Convert a raw Z80 binary into a ZX Spectrum .tap file.")
    ap.add_argument("input", help="raw binary to pack")
    ap.add_argument("-o", "--out", help="output .tap (default: input with .tap)")
    ap.add_argument("--org", type=lambda s: int(s, 0), default=32768,
                    help="load address / entry point (default 32768)")
    ap.add_argument("--name", default=None,
                    help="tape name, max 10 chars (default: derived from file)")
    ap.add_argument("--no-basic", action="store_true",
                    help="write only the CODE block (use LOAD \"\"CODE manually)")
    ap.add_argument("--no-autorun", action="store_true",
                    help="keep the BASIC loader but do not autostart it")
    ap.add_argument("--sna", action="store_true",
                    help="also write a 48K .sna snapshot (instant load)")
    ap.add_argument("--border", type=lambda s: int(s, 0), default=0,
                    help="border colour for the snapshot, 0-7 (default 0)")
    ap.add_argument("--list", action="store_true",
                    help="print the decoded BASIC loader and exit")
    args = ap.parse_args(argv)

    with open(args.input, "rb") as fh:
        code = fh.read()
    if not code:
        ap.error("input file is empty")

    name = args.name
    if name is None:
        name = os.path.splitext(os.path.basename(args.input))[0].upper()
    name = name[:10]

    program, start_line = build_loader(args.org - 1, args.org,
                                       not args.no_autorun)
    if args.list:
        print(list_basic(program))
        return 0

    try:
        tap = make_tap(code, args.org, name,
                       want_basic=not args.no_basic,
                       autostart=not args.no_autorun)
    except ValueError as err:
        ap.error(str(err))

    out_path = args.out or (os.path.splitext(args.input)[0] + ".tap")
    with open(out_path, "wb") as fh:
        fh.write(tap)

    print("%s: %d bytes" % (args.input, len(code)))
    print("%s: %d bytes  (name '%s', org %d, entry %d)" %
          (out_path, len(tap), name, args.org, args.org))
    if not args.no_basic:
        print("BASIC loader:")
        for line in list_basic(program).splitlines():
            print("    " + line)
        if args.no_autorun:
            print("Type RUN after loading the tape.")
        else:
            print("Autostarts on load.")

    if args.sna:
        try:
            sna = make_sna(code, args.org, args.border)
        except ValueError as err:
            ap.error(str(err))
        sna_path = os.path.splitext(out_path)[0] + ".sna"
        with open(sna_path, "wb") as fh:
            fh.write(sna)
        print("%s: %d bytes  (48K snapshot, starts at %d)" %
              (sna_path, len(sna), args.org))
    return 0


if __name__ == "__main__":
    sys.exit(main())
