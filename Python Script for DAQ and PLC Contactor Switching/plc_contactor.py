"""
plc_contactor.py  -  Python replacement for WPLSoft 2.53 "Set On / Set Off"
on the UCT AC microgrid Delta DVP PLC, plus a bridge to Simulink.

WHAT WPLSOFT DOES (from Thando's walkthrough)
  - Connection: RS232, COM7, ASCII, 9600 baud, 7 data bits, Even, 1 stop,
    station address 0. Status bar: "PLC COM: 1 (9600, 7, E, 1)".
  - Each panel switch is a start/stop latch in the ladder, for example
        X0 --[ M5 or Y0 ]--[/ M0 ]-- ( Y0 )        M0 comment: "SG1 OFF"
    Right-click M5 > Set On  closes the switch (Y0 latches through Y0).
    Right-click M0 > Set On  opens it.
  - "Set On / Set Off" is a Modbus ASCII "Force Single Coil" (FC05) message
    sent over the RS-232 cable. This script sends the same message.

WHOLE CHAIN (bidirectional digital twin)
  Simulink --(virtual COM6->COM5)--> Python --(COM7, Modbus ASCII)--> PLC
  Simulink <--(virtual COM5->COM6)-- Python <---(read Y outputs)----- PLC
  Your DAQ (Python -> COM1 -> COM2 -> Simulink) is not touched.

SAFETY RULES BUILT IN
  - Close WPLSoft first. Only one program can own COM7.
  - Only switches in CONTACTORS can be operated.
  - SG22, SG44 and SP1 are synchronising closers. Software can OPEN them
    but never CLOSE them (CLOSE_BLOCKED). Close them by hand after sync.
  - X0 (master enable in every rung) is checked before any command.
  - DRY_RUN = True prints the messages without sending them.
  - Never stop the PLC from Python. WPLSoft asked "Stop PLC?" in the video;
    this script has no stop/run command on purpose.

INSTALL
    pip install pyserial

USAGE
    python plc_contactor.py frame M5 1        show raw message (no PLC needed)
    python plc_contactor.py read X0 Y0 M5     read bits (safe)
    python plc_contactor.py status            state of every mapped switch
    python plc_contactor.py watch M0 64       print bits that change
    python plc_contactor.py on SG1            close SG1 (asks YES)
    python plc_contactor.py off SG1           open SG1 (asks YES)
    python plc_contactor.py listen            show what Simulink sends (no PLC needed)
    python plc_contactor.py bridge            run the Simulink bridge
"""
import sys
import time

import serial

# ---------------------------------------------------------------------------
# 1. PLC link, exactly as in WPLSoft > Communication Setting (video)
# ---------------------------------------------------------------------------
PLC_PORT = "COM7"
BAUD = 9600
BYTESIZE = serial.SEVENBITS
PARITY = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE
STATION = 0              # WPLSoft uses 0. If no reply, try 2 (ladder sets D1121 = K2)
TIMEOUT = 1.0            # s
RETRIES = 3              # WPLSoft "Times of Auto-retry" = 3

DRY_RUN = False           # True = print only, never write to the PLC

# ---------------------------------------------------------------------------
# 2. Switch map, read from the ladder and device comments in WPLSoft.
#    (name, ON bit, OFF bit, status bit)
#    Every row was read off its rung:  ON or Y --[/ OFF ]-- ( Y ).
#    SG1 was tested live from the command line, and SG1-SG6 (first 11 rows)
#    from Simulink through the bridge.
#    Also in the ladder but not mapped here: SG7, SG8, SP4, SP5, SL1,
#    K23 Supp01, K24 Supp02, 1 Phase LE and the rungs for Y40 onwards.
#    The ORDER of this list is the order of the Simulink vectors.
# ---------------------------------------------------------------------------
CONTACTORS = [
    # GENCOR bus: generators
    ("SG1",  "M5",   "M0",  "Y0"),
    ("SG2",  "M50",  "M11", "Y11"),
    ("SG22", "M10",  "M1",  "Y1"),
    ("SG3",  "M15",  "M2",  "Y2"),
    ("SG4",  "M85",  "M19", "Y24"),
    ("SG44", "M20",  "M3",  "Y3"),
    # POOLCOR bus (SP1 = grid utility connection)
    ("SP1",  "M35",  "M7",  "Y6"),
    ("SP2",  "M40",  "M8",  "Y7"),
    ("SP3",  "M45",  "M9",  "Y10"),
    ("SG5",  "M105", "M24", "Y30"),
    ("SG6",  "M110", "M26", "Y31"),
    ("SC1",  "M120", "M28", "Y33"),
    ("SC2",  "M125", "M29", "Y34"),
    # CUSCOR bus: loads
    ("SC3",  "M130", "M31", "Y35"),
    ("SC4",  "M135", "M32", "Y36"),
    ("SC5",  "M140", "M33", "Y37"),
    ("SC6",  "M65",  "M14", "Y20"),
    ("SC7",  "M70",  "M16", "Y21"),
]
CLOSE_BLOCKED = {"SG22", "SG44", "SP1"}
ENABLE_BIT = "X0"        # in series with every rung; must be 1
PULSE_TIME = 0.5         # s, how long a Set On is held before Set Off

# ---------------------------------------------------------------------------
# 3. Simulink bridge (second virtual pair, COM5 <-> COM6)
#    Simulink -> Python: [165 90] + N x uint8  (0 = open, 1 = close, 255 = no command)
#    Python -> Simulink: [165 90] + N x uint8 states + 1 x uint8 X0
#                        (0/1, 255 = unknown)
# ---------------------------------------------------------------------------
BRIDGE_PORT = "COM5"     # Python side. Simulink uses COM6
BRIDGE_BAUD = 115200
HEADER = bytes([0xA5, 0x5A])
STATUS_PERIOD = 0.5      # s
MIN_WRITE_GAP = 1.0      # s per switch
NEW_RUN_GAP = 2.0        # s without frames = Simulink was stopped; next frame is a new baseline


# =========================== Delta addressing ==============================
def modbus_address(dev: str) -> int:
    """Delta DVP device -> Modbus address. X and Y are OCTAL (Y10 = 9th)."""
    kind, num = dev[0].upper(), dev[1:]
    if kind in ("X", "Y"):
        return (0x0400 if kind == "X" else 0x0500) + int(num, 8)
    n = int(num)
    if kind == "M":
        return 0x0800 + n if n < 1536 else 0xB000 + (n - 1536)
    if kind == "S":
        return n
    if kind == "T":
        return 0x0600 + n
    if kind == "C":
        return 0x0E00 + n
    raise ValueError(f"Unsupported device: {dev}")


def device_name(kind: str, n: int) -> str:
    return f"{kind}{oct(n)[2:]}" if kind in ("X", "Y") else f"{kind}{n}"


# =========================== Modbus ASCII driver ===========================
def lrc(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def encode(pdu: bytes) -> bytes:
    body = bytes([STATION]) + pdu
    return b":" + (body + bytes([lrc(body)])).hex().upper().encode() + b"\r\n"


class DeltaPLC:
    """Minimal Modbus ASCII master for a Delta DVP (FC01, FC02, FC05).
    Written directly on pyserial because WPLSoft uses station 0, which
    normal Modbus libraries treat as 'broadcast, no reply'."""

    def __init__(self, port=None):
        self.ser = serial.Serial(port or PLC_PORT, BAUD, bytesize=BYTESIZE, parity=PARITY,
                                 stopbits=STOPBITS, timeout=TIMEOUT)

    def close(self):
        self.ser.close()

    def _transact(self, pdu: bytes) -> bytes:
        last = None
        for _ in range(RETRIES):
            self.ser.reset_input_buffer()
            self.ser.write(encode(pdu))
            line = self.ser.read_until(b"\r\n")
            try:
                if not line.startswith(b":") or not line.endswith(b"\r\n"):
                    raise IOError("no reply (check port, cable, station)")
                raw = bytes.fromhex(line[1:-2].decode())
                if lrc(raw[:-1]) != raw[-1]:
                    raise IOError("bad LRC")
                if raw[1] & 0x80:
                    raise IOError(f"PLC exception code {raw[2]}")
                if raw[1] != pdu[0]:
                    raise IOError("unexpected function code")
                return raw[1:-1]           # function code + data
            except (IOError, ValueError) as err:
                last = err
                time.sleep(0.1)
        raise IOError(f"PLC did not answer after {RETRIES} tries: {last}")

    def read_bits(self, dev: str, count: int = 1) -> list:
        fc = 2 if dev[0].upper() == "X" else 1
        a = modbus_address(dev)
        rsp = self._transact(bytes([fc, a >> 8, a & 0xFF, count >> 8, count & 0xFF]))
        data = rsp[2:2 + rsp[1]]
        return [(data[i // 8] >> (i % 8)) & 1 for i in range(count)]

    def read_bit(self, dev: str) -> int:
        return self.read_bits(dev, 1)[0]

    def set_bit(self, dev: str, value: int) -> None:
        """Same as WPLSoft right-click > Set On / Set Off."""
        if dev[0].upper() == "X":
            raise ValueError("X inputs cannot be written")
        a = modbus_address(dev)
        pdu = bytes([0x05, a >> 8, a & 0xFF, 0xFF if value else 0x00, 0x00])
        if DRY_RUN:
            print(f"[DRY RUN] {dev} <- {value}   {encode(pdu)!r}")
            return
        self._transact(pdu)


# =========================== switching logic ===============================
def lookup(name: str):
    for row in CONTACTORS:
        if row[0].upper() == name.upper():
            return row
    raise KeyError(f"{name} is not in CONTACTORS")


def pulse(plc: DeltaPLC, bit: str) -> None:
    plc.set_bit(bit, 1)
    time.sleep(PULSE_TIME)
    plc.set_bit(bit, 0)


def switch(plc: DeltaPLC, name: str, close: bool, ask: bool = True) -> bool:
    """Close or open one switch the way the ladder expects:
    close = pulse the ON bit (latch holds it), open = pulse the OFF bit."""
    name, on_bit, off_bit, status = lookup(name)
    if close and name in CLOSE_BLOCKED:
        print(f"{name}: CLOSE blocked (synchronising switch). Close it by hand.")
        return False
    if close and not DRY_RUN and plc.read_bit(ENABLE_BIT) != 1:
        print(f"{name}: {ENABLE_BIT} is OFF, ladder will not latch. Not sent.")
        return False
    if ask:
        word = "CLOSE" if close else "OPEN"
        if input(f"{word} {name}? Type YES: ").strip() != "YES":
            print("Cancelled.")
            return False
    if close:
        plc.set_bit(off_bit, 0)          # make sure the stop bit is released
        pulse(plc, on_bit)
    else:
        pulse(plc, off_bit)
    if not DRY_RUN:
        time.sleep(0.2)
        print(f"{name}: {status} = {plc.read_bit(status)}")
    return True


def read_status(plc: DeltaPLC) -> list:
    """Y state of every switch in CONTACTORS order, then X0. 255 = unknown.
    All Y outputs are read in ONE message so the PLC port stays free."""
    ys = [int(row[3][1:], 8) for row in CONTACTORS]
    try:
        bits = plc.read_bits("Y0", max(ys) + 1)
        out = [bits[i] for i in ys]
    except Exception:
        out = [255] * len(ys)
    try:
        out.append(plc.read_bit(ENABLE_BIT))
    except Exception:
        out.append(255)
    return out


def watch(plc: DeltaPLC, start: str, count: int) -> None:
    kind = start[0].upper()
    first = int(start[1:], 8) if kind in ("X", "Y") else int(start[1:])
    old = None
    print(f"Watching {count} bits from {start}. Ctrl+C to stop.")
    try:
        while True:
            bits = plc.read_bits(start, count)
            if old is not None:
                for i, (a, b) in enumerate(zip(old, bits)):
                    if a != b:
                        print(f"{time.strftime('%H:%M:%S')}  "
                              f"{device_name(kind, first + i)}: {a} -> {b}")
            old = bits
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("Stopped.")


# =========================== Simulink bridge ===============================
def read_frame(port: serial.Serial, n: int, buf: bytearray):
    """Newest complete command frame in the buffer, or None."""
    buf.extend(port.read(port.in_waiting or 1))
    latest = None
    while True:
        i = buf.find(HEADER)
        if i < 0:
            del buf[:-1]
            return latest
        if len(buf) - i < 2 + n:
            del buf[:i]
            return latest
        latest = list(buf[i + 2:i + 2 + n])
        del buf[:i + 2 + n]


def listen() -> None:
    """Print the command frames arriving from Simulink. Opens only the
    bridge port, so it checks the Simulink side without a PLC."""
    n = len(CONTACTORS)
    link = serial.Serial(BRIDGE_PORT, BRIDGE_BAUD, timeout=0.05)
    buf, previous = bytearray(), None
    print(f"Listening on {BRIDGE_PORT} for [165 90] + {n} bytes. Ctrl+C to stop.")
    try:
        while True:
            frame = read_frame(link, n, buf)
            if frame is None or frame == previous:
                continue
            if previous is None:
                print("First frame:", frame)
            else:
                for row, old, new in zip(CONTACTORS, previous, frame):
                    if old != new:
                        print(f"{time.strftime('%H:%M:%S')}  {row[0]}: {old} -> {new}")
            if any(v not in (0, 1, 255) for v in frame):
                print("  Values other than 0, 1, 255: is the Simulink signal uint8?")
            previous = frame
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        link.close()


def bridge() -> None:
    """Act on CHANGES in the Simulink command vector only. The first frame
    of every simulation run is the baseline and switches nothing."""
    n = len(CONTACTORS)
    plc = DeltaPLC()
    link = serial.Serial(BRIDGE_PORT, BRIDGE_BAUD, timeout=0.05, write_timeout=0.2)
    buf, previous, last_write, next_status = bytearray(), None, {}, 0.0
    last_frame = 0.0
    print(f"Bridge: PLC {PLC_PORT}, Simulink {BRIDGE_PORT}, {n} switches, "
          f"DRY_RUN={DRY_RUN}. Ctrl+C to stop.")
    try:
        while True:
            desired = read_frame(link, n, buf)
            if desired is not None:
                if time.monotonic() - last_frame > NEW_RUN_GAP:
                    previous = None              # new simulation run
                last_frame = time.monotonic()
                if previous is None:
                    print("Baseline from Simulink:", desired)
                else:
                    for k, (old, new) in enumerate(zip(previous, desired)):
                        if new == old or new not in (0, 1):
                            continue
                        name, now = CONTACTORS[k][0], time.monotonic()
                        if now - last_write.get(name, 0) < MIN_WRITE_GAP:
                            desired[k] = old     # too soon: retry on a later frame
                            continue
                        try:
                            if switch(plc, name, bool(new), ask=False):
                                last_write[name] = now
                                print(f"{time.strftime('%H:%M:%S')}  {name} -> {new}")
                        except IOError as err:
                            print(f"{name}: {err}")
                previous = desired
            if time.monotonic() >= next_status:
                try:
                    link.write(HEADER + bytes(read_status(plc)))
                except serial.SerialTimeoutException:
                    pass                 # Simulink is not reading; keep going
                next_status = time.monotonic() + STATUS_PERIOD
    except KeyboardInterrupt:
        print("Bridge stopped.")
    finally:
        link.close()
        plc.close()


# =========================== command line ==================================
def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        return
    cmd, args = sys.argv[1].lower(), sys.argv[2:]
    if cmd == "frame":
        a = modbus_address(args[0])
        print(encode(bytes([5, a >> 8, a & 0xFF, 0xFF if int(args[1]) else 0, 0])))
        return
    if cmd == "listen":
        listen()
        return
    if cmd == "bridge":
        bridge()
        return
    plc = DeltaPLC()
    try:
        if cmd == "read":
            for dev in args:
                print(f"{dev.upper()} = {plc.read_bit(dev)}")
        elif cmd == "status":
            names = [row[0] for row in CONTACTORS] + [ENABLE_BIT]
            for name, s in zip(names, read_status(plc)):
                print(f"{name:5s} {'unknown' if s == 255 else s}")
        elif cmd == "watch":
            watch(plc, args[0], int(args[1]) if len(args) > 1 else 32)
        elif cmd in ("on", "off"):
            switch(plc, args[0], cmd == "on")
        else:
            print(__doc__)
    finally:
        plc.close()


if __name__ == "__main__":
    main()
