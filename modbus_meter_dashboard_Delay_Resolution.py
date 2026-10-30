"""
Live dashboard for all 11 meters, with a dropdown to select which one to view.

Efficiency note: each meter's 14 values sit in one contiguous 28-register
block (e.g. meter 8 spans addresses 100-127), so each meter is read in a
SINGLE Modbus request rather than 14 separate ones, then sliced locally.

A background thread reads whichever meter you currently have selected on
EVERY loop (so it updates fast, independent of how many meters exist), and
rotates through the other meters one at a time in between so they still stay
reasonably fresh in the background even though nobody's actively watching them.

CONFIRMED so far: DECODE_TYPE=uint32, WORD_ORDER=little, scale=0.1 for U12
(gave a sensible 387.1V). The 0.1 scale below is applied to every field as a
starting assumption — spot-check Frequency, Current, and especially P/Q/S
(kW/kVAr/kVA, which haven't been individually verified yet) against the HMI's
own display once this is running, and adjust per-field scale if any look off
by a factor of 10/100/1000.
"""

import tkinter as tk
from tkinter import ttk, filedialog
import threading
import queue
import time
import csv
import os
import struct

from pymodbus.client import ModbusTcpClient
import serial

# ---------------- CONFIG ----------------
HMI_IP = "192.168.0.1"       # <-- confirm this matches your working test
HMI_PORT = 502
UNIT_ID = 1
LOOP_DELAY = 0.3               # short pacing delay between loops (not a full-round delay anymore)

DECODE_TYPE = "uint32"
WORD_ORDER = "little"

# ---- Serial streaming to Simulink (virtual COM port) ----
# Set to the COM port name your virtual-serial pair gives Python (e.g. "COM1")
# to continuously stream every meter's readings to a Simulink Serial Receive
# block over the other end of the pair (COM2 in the model). Leave as None to
# disable streaming and keep the dashboard's normal GUI-priority polling
# (whichever meter you're viewing refreshes fastest).
#
# Frame layout, one snapshot of ALL meters per frame (little-endian):
#   [0xAA, 0x55] + 154 x float32  ->  2 + 616 = 618 bytes total
#   Values are channel-major: SNAPSHOT_CHANNELS[0]'s 14 FIELDS values (in
#   FIELDS order), then SNAPSHOT_CHANNELS[1]'s 14, ... This matches the SLX's
#   Demux11, whose 154 outputs are wired 1:1 to 154 fixed Goto blocks in this
#   exact order (c1U12, c1U23, ..., c1S, c2U12, ..., c16S) — there is no
#   per-meter routing in the model, so every frame must carry every meter's
#   latest reading, not just the one that changed.
#
# Configure the Serial Receive block to match: Header = [170 85] (uint8),
# Data type = single, Data size = [154 1], Input Format = Column major,
# "Action when data is not available" = Output last received value.
SERIAL_PORT = "COM1"
SERIAL_BAUD = 115200
SERIAL_HEADER = bytes([0xAA, 0x55])

# Channel order the SLX's Demux11 -> Goto wiring expects, verified against
# the model (do not reorder without re-checking the model's Goto tags).
SNAPSHOT_CHANNELS = ["C1", "C2", "C5", "C7", "C8", "C9", "C10", "C11", "C12", "C14", "C16"]

# label, offset (in words) within a meter's 28-word block, unit, scale
FIELDS = [
    ("U12 (L1-L2)",        0,  "V",    0.01),
    ("U23 (L2-L3)",        2,  "V",    0.01),
    ("U31 (L3-L1)",        4,  "V",    0.01),
    ("Power Factor",       6,  "",     0.01),
    ("V1 (L1-N)",          8,  "V",    0.01),
    ("V2 (L2-N)",          10, "V",    0.01),
    ("V3 (L3-N)",          12, "V",    0.01),
    ("Frequency",          14, "Hz",   0.01),
    ("Current 1",          16, "A",    0.001),
    ("Current 2",          18, "A",    0.001),
    ("Current 3",          20, "A",    0.001),
    ("Active Power (P)",   22, "kW",   0.01),
    ("Reactive Power (Q)", 24, "kVAr", 0.01),
    ("Apparent Power (S)", 26, "kVA",  0.01),
]
REGISTERS_PER_METER = 28  # 14 fields x 2 words each

# meter name -> base address (from your register map)
METERS = {
    "C1":  325,
    "C2":  353,
    "C5":  381,
    "C7":  157,
    "C8":  185,
    "C12":  101,
    "C14":  129,
    "C9": 269,
    "C10": 213,
    "C11": 241,
    "C16": 297,
}
# -----------------------------------------

_DATATYPE_MAP = {
    "float32": ModbusTcpClient.DATATYPE.FLOAT32,
    "uint32": ModbusTcpClient.DATATYPE.UINT32,
    "int32": ModbusTcpClient.DATATYPE.INT32,
}


def decode_dw(client, registers, decode_type):
    return client.convert_from_registers(registers, _DATATYPE_MAP[decode_type], word_order=WORD_ORDER)


def build_snapshot_frame(cache, fields, channel_order):
    """Pack the latest cached reading for every meter into one 154-value
    snapshot frame, channel-major, matching the SLX's fixed Demux/Goto order."""
    values = []
    for meter_name in channel_order:
        readings = cache.get(meter_name, {}).get("readings", {})
        for label, *_ in fields:
            values.append(readings.get(label) or 0.0)
    return SERIAL_HEADER + struct.pack(f"<{len(values)}f", *values)


class MeterPoller(threading.Thread):
    """
    Background thread. Reads whichever meter is currently selected (priority_meter)
    on EVERY loop, so the one you're actually watching updates fast regardless of
    how many total meters exist. In between, it reads one other meter from a
    rotation so the rest keep refreshing too, just less frequently since nobody's
    watching them in real time.
    """

    def __init__(self, ip, port, unit_id, meters, fields, loop_delay, out_queue):
        super().__init__(daemon=True)
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.meters = meters
        self.fields = fields
        self.loop_delay = loop_delay
        self.out_queue = out_queue
        self._stop = threading.Event()
        # short timeout so one slow/failing request can't stall everything for seconds
        self.client = ModbusTcpClient(self.ip, port=self.port, timeout=1.0)
        self.priority_meter = list(meters.keys())[0]  # updated by the GUI when selection changes
        self.equal_priority = False  # set True by the GUI while an all-meters logging session runs
        self._other_index = 0

    def _read_and_publish(self, meter_name):
        base_address = self.meters[meter_name]
        readings = {}
        status = "ok"
        try:
            result = self.client.read_holding_registers(
                base_address, count=REGISTERS_PER_METER, device_id=self.unit_id
            )
            if result.isError():
                status = "error"
                for label, _off, _unit, _scale in self.fields:
                    readings[label] = None
            else:
                for label, offset, _unit, scale in self.fields:
                    pair = result.registers[offset:offset + 2]
                    raw_value = decode_dw(self.client, pair, DECODE_TYPE)
                    readings[label] = raw_value * scale
        except Exception:
            status = "disconnected"
            for label, _off, _unit, _scale in self.fields:
                readings[label] = None

        self.out_queue.put({
            "meter": meter_name,
            "readings": readings,
            "status": status,
            "time": time.strftime("%H:%M:%S"),
        })

    def run(self):
        meter_names = list(self.meters.keys())
        while not self._stop.is_set():
            if not self.client.connected:
                self.client.connect()

            if self.equal_priority:
                # even round-robin across all meters — used while logging ALL
                # meters, so every one gets refreshed at the same rate
                name = meter_names[self._other_index % len(meter_names)]
                self._read_and_publish(name)
                self._other_index += 1
            else:
                # 1. always refresh whichever meter is currently being viewed
                priority = self.priority_meter
                self._read_and_publish(priority)

                # 2. also refresh one other meter each loop, rotating through the rest
                others = [m for m in meter_names if m != priority]
                if others and not self._stop.is_set():
                    name = others[self._other_index % len(others)]
                    self._read_and_publish(name)
                    self._other_index += 1

            time.sleep(self.loop_delay)

    def stop(self):
        self._stop.set()
        self.client.close()


class MeterDashboard(tk.Tk):
    def __init__(self, meters, fields):
        super().__init__()
        self.title("Live Meter Readings")
        self.geometry("440x620")
        self.configure(padx=15, pady=15)

        self.meters = meters
        self.fields = fields
        self.units = {label: unit for label, _off, unit, _scale in fields}

        # cache of latest readings per meter, so switching the dropdown shows
        # the last known value immediately instead of going blank
        self.cache = {name: {"readings": {}, "status": "waiting", "time": "--"} for name in meters}

        # CSV logging state — logging targets whichever meter was selected
        # when "Start Logging" was clicked, independent of what you browse
        # to afterward, so you can keep looking at other meters while one
        # logs quietly in the background.
        self.is_logging = False
        self.log_file_handle = None
        self.log_writer = None
        self.logging_meter = None   # a specific meter name, or None meaning "log every meter"
        self.log_all_var = tk.BooleanVar(value=False)
        self.log_status_var = tk.StringVar(value="Not logging")

        self.selected_meter = tk.StringVar(value=list(meters.keys())[0])
        self.status_var = tk.StringVar(value="Connecting...")
        self.value_vars = {label: tk.StringVar(value="--") for label, _, _, _ in fields}

        selector_frame = ttk.Frame(self)
        selector_frame.grid(row=0, column=0, columnspan=2, pady=(0, 10), sticky="ew")
        ttk.Label(selector_frame, text="Meter:", font=("Segoe UI", 11)).pack(side="left", padx=(0, 8))
        self.combo = ttk.Combobox(
            selector_frame, textvariable=self.selected_meter,
            values=list(meters.keys()), state="readonly", width=20
        )
        self.combo.pack(side="left")
        self.combo.bind("<<ComboboxSelected>>", lambda e: self.on_meter_selected())

        ttk.Label(self, textvariable=self.status_var, foreground="gray").grid(
            row=1, column=0, columnspan=2, pady=(0, 10)
        )

        log_frame = ttk.Frame(self)
        log_frame.grid(row=2, column=0, columnspan=2, pady=(0, 5), sticky="ew")
        self.start_log_btn = ttk.Button(log_frame, text="Start Logging", command=self.start_logging)
        self.start_log_btn.pack(side="left", padx=(0, 8))
        self.stop_log_btn = ttk.Button(log_frame, text="Stop Logging", command=self.stop_logging, state="disabled")
        self.stop_log_btn.pack(side="left", padx=(0, 8))
        self.log_all_check = ttk.Checkbutton(log_frame, text="Log all meters", variable=self.log_all_var)
        self.log_all_check.pack(side="left")

        ttk.Label(self, textvariable=self.log_status_var, foreground="gray").grid(
            row=3, column=0, columnspan=2, pady=(0, 15)
        )

        for i, (label, _off, _unit, _scale) in enumerate(fields, start=4):
            ttk.Label(self, text=label + ":").grid(row=i, column=0, sticky="w", pady=4)
            ttk.Label(self, textvariable=self.value_vars[label], font=("Segoe UI", 10, "bold")).grid(
                row=i, column=1, sticky="e", pady=4
            )

        self.serial_status_var = tk.StringVar(value="Serial streaming disabled")
        ttk.Label(self, textvariable=self.serial_status_var, foreground="gray").grid(
            row=len(fields) + 4, column=0, columnspan=2, pady=(10, 0)
        )

        self.queue = queue.Queue()
        self.poller = MeterPoller(HMI_IP, HMI_PORT, UNIT_ID, meters, fields, LOOP_DELAY, self.queue)
        self.poller.start()

        self.serial_conn = None
        if SERIAL_PORT:
            try:
                self.serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
                # streaming needs every meter refreshed evenly, not weighted
                # toward whichever one the GUI happens to have selected
                self.poller.equal_priority = True
                self.serial_status_var.set(f"Streaming to Simulink on {SERIAL_PORT}")
            except serial.SerialException as exc:
                self.serial_status_var.set(f"Serial port {SERIAL_PORT} unavailable: {exc}")

        self.after(200, self.poll_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def poll_queue(self):
        try:
            while True:
                data = self.queue.get_nowait()
                self.cache[data["meter"]] = data
                if data["meter"] == self.selected_meter.get():
                    self.update_display(data)
                if self.is_logging and (self.logging_meter is None or data["meter"] == self.logging_meter):
                    self.write_log_row(data)
                if self.serial_conn is not None:
                    self.write_serial_frame()
        except queue.Empty:
            pass
        self.after(200, self.poll_queue)

    def write_log_row(self, data):
        row = [data["time"], data["meter"]] + [
            data["readings"].get(label, "") for label, _off, _unit, _scale in self.fields
        ]
        self.log_writer.writerow(row)
        self.log_file_handle.flush()  # flush after every row so data survives a crash/power loss

    def write_serial_frame(self):
        try:
            self.serial_conn.write(build_snapshot_frame(self.cache, self.fields, SNAPSHOT_CHANNELS))
        except serial.SerialException:
            self.serial_status_var.set("Serial write failed — check connection")

    def start_logging(self):
        log_all = self.log_all_var.get()
        target_name = "All_Meters" if log_all else self.selected_meter.get()
        default_name = f"{target_name.replace(' ', '_')}_{time.strftime('%Y%m%d_%H%M%S')}.csv"

        os.makedirs("logs", exist_ok=True)
        path = filedialog.asksaveasfilename(
            initialdir="logs",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            title="Save log as",
        )
        if not path:
            return  # user cancelled

        self.log_file_handle = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log_file_handle)
        header = ["timestamp", "meter"] + [
            f"{label} ({unit})" if unit else label for label, _off, unit, _scale in self.fields
        ]
        self.log_writer.writerow(header)
        self.log_file_handle.flush()

        self.logging_meter = None if log_all else self.selected_meter.get()
        self.is_logging = True
        if log_all:
            self.poller.equal_priority = True
        self.start_log_btn.config(state="disabled")
        self.stop_log_btn.config(state="normal")
        self.log_all_check.config(state="disabled")  # lock the mode while a session is running

        if log_all:
            self.log_status_var.set(f"Logging ALL meters to {os.path.basename(path)}")
        else:
            self.log_status_var.set(f"Logging {target_name} to {os.path.basename(path)}")

    def stop_logging(self):
        if self.log_file_handle:
            self.log_file_handle.close()
        saved_path = self.log_file_handle.name if self.log_file_handle else None
        self.log_file_handle = None
        self.log_writer = None
        self.is_logging = False
        self.logging_meter = None
        # only drop back to priority-weighted polling if serial streaming
        # (which also needs even round-robin) isn't the one relying on it
        self.poller.equal_priority = self.serial_conn is not None
        self.start_log_btn.config(state="normal")
        self.stop_log_btn.config(state="disabled")
        self.log_all_check.config(state="normal")
        if saved_path:
            self.log_status_var.set(f"Saved: {os.path.basename(saved_path)}")
        else:
            self.log_status_var.set("Not logging")

    def on_meter_selected(self):
        self.poller.priority_meter = self.selected_meter.get()
        self.refresh_from_cache()

    def refresh_from_cache(self):
        data = self.cache.get(self.selected_meter.get())
        if data:
            self.update_display(data)

    def update_display(self, data):
        for label, value in data["readings"].items():
            if value is None:
                self.value_vars[label].set("--")
            else:
                unit = self.units.get(label, "")
                self.value_vars[label].set(f"{value:.2f} {unit}".strip())

        status_text = {
            "ok": f"Connected — last update {data['time']}",
            "error": f"Some registers failed — {data['time']}",
            "disconnected": "Disconnected — retrying...",
            "waiting": "Waiting for first reading...",
        }.get(data["status"], data["status"])
        self.status_var.set(status_text)

    def on_close(self):
        if self.is_logging:
            self.stop_logging()
        if self.serial_conn is not None:
            self.serial_conn.close()
        self.poller.stop()
        self.destroy()


if __name__ == "__main__":
    app = MeterDashboard(METERS, FIELDS)
    app.mainloop()
