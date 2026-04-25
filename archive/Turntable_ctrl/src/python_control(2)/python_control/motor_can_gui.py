#!/usr/bin/env python3
from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, scrolledtext, ttk
from typing import Any

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:
    serial = None
    list_ports = None


@dataclass(frozen=True)
class SerialBridgeConfig:
    port: str
    baudrate: int
    timeout: float
    open_delay_s: float


@dataclass(frozen=True)
class SerialCommand:
    position: float = 0.0
    kp: float = 2.0
    velocity: float = 0.0
    kd: float = 0.1


def require_pyserial() -> None:
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")


def detect_default_port(preferred: str = "COM13") -> str:
    if list_ports is None:
        return preferred
    ports = {port.device.upper(): port.device for port in list_ports.comports()}
    if preferred.upper() in ports:
        return ports[preferred.upper()]
    if ports:
        return ports[sorted(ports)[0]]
    return preferred


def list_available_ports() -> list[str]:
    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


def format_bridge_command(command: SerialCommand) -> str:
    return (
        f"{command.position:.3f},"
        f"{command.kp:.3f},"
        f"{command.velocity:.3f},"
        f"{command.kd:.3f}\n"
    )


def open_serial_port(config: SerialBridgeConfig) -> Any:
    require_pyserial()
    kwargs = {
        "baudrate": config.baudrate,
        "timeout": config.timeout,
        "write_timeout": 1.0,
    }
    if "://" in config.port:
        conn = serial.serial_for_url(config.port, **kwargs)
    else:
        conn = serial.Serial(config.port, **kwargs)

    for attr in ("dtr", "rts"):
        try:
            setattr(conn, attr, False)
        except Exception:
            pass

    for method_name in ("reset_input_buffer", "reset_output_buffer"):
        method = getattr(conn, method_name, None)
        if callable(method):
            try:
                method()
            except Exception:
                pass
    return conn


class SerialBridgeWorker(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.command_queue: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self.event_queue: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue()
        self.stop_event = threading.Event()

        self.connection: Any = None
        self.connected = False
        self.current_command = SerialCommand()
        self.continuous_send = False
        self.send_hz = 20.0
        self.next_send_time = 0.0

        self.tx_count = 0
        self.rx_count = 0
        self.last_tx_time = 0.0
        self.last_rx_time = 0.0

    def emit(self, event_type: str, **payload: Any) -> None:
        self.event_queue.put((event_type, payload))

    def log(self, message: str) -> None:
        self.emit("log", message=message)

    def submit(self, action: str, **payload: Any) -> None:
        self.command_queue.put((action, payload))

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        self.emit_stats()
        while not self.stop_event.is_set():
            self._process_commands()
            self._send_periodic_if_needed()
            self._read_once()
            self._emit_stats_if_needed()
        self._disconnect_internal(notify=False)

    def emit_stats(self) -> None:
        self.emit(
            "stats",
            connected=self.connected,
            tx_count=self.tx_count,
            rx_count=self.rx_count,
            last_tx_time=self.last_tx_time,
            last_rx_time=self.last_rx_time,
        )

    def _emit_stats_if_needed(self) -> None:
        now = time.time()
        last_emit = getattr(self, "_last_stats_emit", 0.0)
        if now - last_emit >= 0.25:
            self._last_stats_emit = now
            self.emit_stats()

    def _process_commands(self) -> None:
        while True:
            try:
                action, payload = self.command_queue.get_nowait()
            except queue.Empty:
                return

            try:
                if action == "connect":
                    self._connect_internal(payload["config"])
                elif action == "disconnect":
                    self._disconnect_internal(notify=True)
                elif action == "send_once":
                    self.current_command = payload["command"]
                    self._send_command(self.current_command)
                elif action == "set_continuous":
                    self.current_command = payload["command"]
                    self.send_hz = payload["send_hz"]
                    self.continuous_send = payload["enabled"]
                    self.next_send_time = 0.0
                    state = "started" if self.continuous_send else "stopped"
                    self.log(f"Continuous send {state} at {self.send_hz:.1f} Hz.")
                elif action == "update_command":
                    self.current_command = payload["command"]
                elif action == "send_relax":
                    relax = SerialCommand(position=0.0, kp=0.0, velocity=0.0, kd=0.0)
                    self.current_command = relax
                    self._send_command(relax)
                    self.log("Sent relax command: 0,0,0,0")
            except Exception as exc:
                self.emit("error", message=str(exc))

    def _connect_internal(self, config: SerialBridgeConfig) -> None:
        self._disconnect_internal(notify=False)
        connection = open_serial_port(config)
        self.connection = connection
        if config.open_delay_s > 0:
            time.sleep(config.open_delay_s)

        self.connected = True
        self.continuous_send = False
        self.next_send_time = 0.0
        self.tx_count = 0
        self.rx_count = 0
        self.last_tx_time = 0.0
        self.last_rx_time = 0.0
        self.emit("connected", port=config.port, baudrate=config.baudrate)
        self.log(
            "Connected serial bridge: port={0} baudrate={1} timeout={2:.3f}s".format(
                config.port,
                config.baudrate,
                config.timeout,
            )
        )

    def _disconnect_internal(self, notify: bool) -> None:
        self.connected = False
        self.continuous_send = False
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
        self.connection = None
        if notify:
            self.emit("disconnected")
            self.log("Disconnected serial bridge.")

    def _send_command(self, command: SerialCommand) -> None:
        if not self.connected or self.connection is None:
            raise RuntimeError("Serial bridge is not connected.")
        line = format_bridge_command(command)
        self.connection.write(line.encode("ascii"))
        flush = getattr(self.connection, "flush", None)
        if callable(flush):
            flush()
        self.tx_count += 1
        self.last_tx_time = time.time()
        self.emit("tx", line=line.rstrip("\n"), timestamp=self.last_tx_time)

    def _send_periodic_if_needed(self) -> None:
        if not self.connected or self.connection is None or not self.continuous_send:
            return
        now = time.time()
        if now < self.next_send_time:
            return
        self._send_command(self.current_command)
        self.next_send_time = now + (1.0 / max(self.send_hz, 1.0))

    def _read_once(self) -> None:
        if not self.connected or self.connection is None:
            time.sleep(0.01)
            return

        try:
            raw = self.connection.readline()
        except Exception as exc:
            self.emit("error", message=f"Serial read failed: {exc}")
            self._disconnect_internal(notify=True)
            return

        if not raw:
            time.sleep(0.005)
            return

        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        self.rx_count += 1
        self.last_rx_time = time.time()
        self.emit("serial_line", line=line, timestamp=self.last_rx_time)


class MotorSerialGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Motor Serial Bridge")
        self.root.geometry("1180x820")
        self.root.minsize(1080, 720)

        self.worker = SerialBridgeWorker()
        self.worker.start()

        self.connected = False
        self.tx_count = 0
        self.rx_count = 0
        self.last_tx_time = 0.0
        self.last_rx_time = 0.0
        self.last_board_line = ""

        self._build_variables()
        self._build_layout()
        self._bind_variable_updates()
        self._poll_events()
        self._refresh_status()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_variables(self) -> None:
        default_port = detect_default_port("COM13")
        self.port_var = tk.StringVar(value=default_port)
        self.baudrate_var = tk.StringVar(value="115200")
        self.timeout_var = tk.StringVar(value="0.02")
        self.open_delay_var = tk.StringVar(value="2.0")

        self.position_var = tk.DoubleVar(value=0.0)
        self.kp_var = tk.DoubleVar(value=2.0)
        self.velocity_var = tk.DoubleVar(value=0.0)
        self.kd_var = tk.DoubleVar(value=0.1)
        self.send_hz_var = tk.DoubleVar(value=20.0)
        self.continuous_send_var = tk.BooleanVar(value=False)
        self.auto_send_var = tk.BooleanVar(value=False)

        self.status_var = tk.StringVar(value="Disconnected")
        self.detail_var = tk.StringVar(value="Connect to the controller board over serial.")
        self.port_info_var = tk.StringVar(value=f"port={default_port} baud=115200")
        self.counter_var = tk.StringVar(value="TX: 0   RX: 0")
        self.last_board_line_var = tk.StringVar(value="--")
        self.last_command_var = tk.StringVar(value=format_bridge_command(self._build_command()).strip())

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="nsew")
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=1)

        self._build_connection_frame(top).grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self._build_status_frame(top).grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        middle = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        middle.grid(row=1, column=0, sticky="nsew")
        middle.columnconfigure(0, weight=3)
        middle.columnconfigure(1, weight=2)

        self._build_control_frame(middle).grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self._build_protocol_frame(middle).grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        bottom = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        bottom.grid(row=2, column=0, sticky="nsew", padx=8, pady=(0, 8))

        log_frame = ttk.LabelFrame(bottom, text="Serial Log", padding=8)
        help_frame = ttk.LabelFrame(bottom, text="Usage Notes", padding=8)
        bottom.add(log_frame, weight=3)
        bottom.add(help_frame, weight=2)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=18, wrap=tk.WORD, font=("Consolas", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

        help_text = scrolledtext.ScrolledText(help_frame, height=18, wrap=tk.WORD, font=("Consolas", 10))
        help_text.pack(fill=tk.BOTH, expand=True)
        help_text.insert(
            "1.0",
            "\n".join(
                [
                    "This GUI talks to the SF_Control firmware over serial, not direct CAN.",
                    "",
                    "Firmware protocol:",
                    "  position,kp,velocity,kd\\n",
                    "",
                    "Examples that match your working serial monitor:",
                    "  0.000,2.000,0.000,0.100",
                    "  0.500,2.000,0.000,0.100",
                    "  0.100,1.000,0.000,0.050",
                    "",
                    "Recommended workflow:",
                    "1. Connect to the board serial port.",
                    "2. Set target position / gains.",
                    "3. Click Send Once.",
                    "4. Use Continuous Send only if you want repeated updates.",
                    "",
                    "The firmware enables the motor in setup(), so this GUI does not send CAN enable frames directly.",
                ]
            ),
        )
        help_text.configure(state=tk.DISABLED)

    def _build_connection_frame(self, parent: ttk.Frame) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text="Serial Bridge", padding=8)
        for idx in range(4):
            frame.columnconfigure(idx, weight=1)

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w")
        port_combo = ttk.Combobox(frame, textvariable=self.port_var, values=list_available_ports(), width=12)
        port_combo.grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(frame, text="Baudrate").grid(row=0, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.baudrate_var, width=12).grid(row=0, column=3, sticky="ew", padx=4)

        ttk.Label(frame, text="Read timeout (s)").grid(row=1, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.timeout_var, width=12).grid(row=1, column=1, sticky="ew", padx=4)

        ttk.Label(frame, text="Open delay (s)").grid(row=1, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.open_delay_var, width=12).grid(row=1, column=3, sticky="ew", padx=4)

        button_row = ttk.Frame(frame)
        button_row.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        for idx in range(4):
            button_row.columnconfigure(idx, weight=1)

        ttk.Button(button_row, text="Connect", command=self.connect_bridge).grid(row=0, column=0, padx=3, sticky="ew")
        ttk.Button(button_row, text="Disconnect", command=self.disconnect_bridge).grid(row=0, column=1, padx=3, sticky="ew")
        ttk.Button(button_row, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=2, padx=3, sticky="ew")
        ttk.Button(button_row, text="Relax 0,0,0,0", command=self.send_relax).grid(row=0, column=3, padx=3, sticky="ew")
        return frame

    def _build_status_frame(self, parent: ttk.Frame) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text="Status", padding=8)
        frame.columnconfigure(1, weight=1)

        self.status_badge = tk.Label(
            frame,
            textvariable=self.status_var,
            bg="#808080",
            fg="white",
            font=("Consolas", 12, "bold"),
            padx=12,
            pady=8,
        )
        self.status_badge.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        ttk.Label(frame, text="Connection").grid(row=1, column=0, sticky="nw")
        ttk.Label(frame, textvariable=self.port_info_var, wraplength=420, justify=tk.LEFT).grid(row=1, column=1, sticky="w")

        ttk.Label(frame, text="Details").grid(row=2, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(frame, textvariable=self.detail_var, wraplength=420, justify=tk.LEFT).grid(row=2, column=1, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="Counters").grid(row=3, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(frame, textvariable=self.counter_var).grid(row=3, column=1, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="Last board line").grid(row=4, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(frame, textvariable=self.last_board_line_var, wraplength=420, justify=tk.LEFT).grid(row=4, column=1, sticky="w", pady=(8, 0))
        return frame

    def _build_control_frame(self, parent: ttk.Frame) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text="Command", padding=8)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        self._add_control_row(frame, 0, "Position (rad)", self.position_var, -6.0, 6.0)
        self._add_control_row(frame, 1, "Kp", self.kp_var, 0.0, 20.0)
        self._add_control_row(frame, 2, "Velocity (rad/s)", self.velocity_var, -10.0, 10.0)
        self._add_control_row(frame, 3, "Kd", self.kd_var, 0.0, 2.0)
        self._add_control_row(frame, 4, "Repeat rate (Hz)", self.send_hz_var, 1.0, 50.0, digits=1, scale_resolution=1.0)

        action_row = ttk.Frame(frame)
        action_row.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        for idx in range(4):
            action_row.columnconfigure(idx, weight=1)

        ttk.Button(action_row, text="Send Once", command=self.send_once).grid(row=0, column=0, padx=3, sticky="ew")
        ttk.Checkbutton(
            action_row,
            text="Continuous Send",
            variable=self.continuous_send_var,
            command=self.toggle_continuous_send,
        ).grid(row=0, column=1, padx=3, sticky="ew")
        ttk.Checkbutton(
            action_row,
            text="Auto Send On Change",
            variable=self.auto_send_var,
        ).grid(row=0, column=2, padx=3, sticky="ew")
        ttk.Button(action_row, text="Stop Repeat", command=self.stop_repeat).grid(row=0, column=3, padx=3, sticky="ew")
        return frame

    def _build_protocol_frame(self, parent: ttk.Frame) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text="Protocol Preview", padding=8)
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Outgoing line").grid(row=0, column=0, sticky="nw")
        ttk.Label(frame, textvariable=self.last_command_var, wraplength=340, justify=tk.LEFT).grid(row=0, column=1, sticky="w")

        ttk.Label(frame, text="Firmware expectation").grid(row=1, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(
            frame,
            text="position,kp,velocity,kd\\n",
            justify=tk.LEFT,
        ).grid(row=1, column=1, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="Current mapping").grid(row=2, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(
            frame,
            text="The SF_Control firmware stores the latest serial command and keeps sending MIT CAN commands in its own loop.",
            wraplength=340,
            justify=tk.LEFT,
        ).grid(row=2, column=1, sticky="w", pady=(8, 0))
        return frame

    def _add_control_row(
        self,
        parent: ttk.LabelFrame,
        row: int,
        label: str,
        variable: tk.DoubleVar,
        minimum: float,
        maximum: float,
        digits: int = 3,
        scale_resolution: float = 0.01,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=variable, width=10).grid(row=row, column=1, sticky="ew", padx=(4, 8), pady=4)
        scale = tk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            variable=variable,
            orient=tk.HORIZONTAL,
            resolution=scale_resolution,
            showvalue=False,
            length=280,
        )
        scale.grid(row=row, column=2, columnspan=2, sticky="ew", pady=2)
        parent.grid_columnconfigure(2, weight=1)
        parent.grid_columnconfigure(3, weight=1)

    def _bind_variable_updates(self) -> None:
        self._pending_update_job: str | None = None
        tracked = (
            self.position_var,
            self.kp_var,
            self.velocity_var,
            self.kd_var,
            self.send_hz_var,
        )
        for variable in tracked:
            variable.trace_add("write", self._on_command_variable_changed)

    def _on_command_variable_changed(self, *_args: Any) -> None:
        if self._pending_update_job is not None:
            self.root.after_cancel(self._pending_update_job)
        self._pending_update_job = self.root.after(100, self._push_command_update)

    def _push_command_update(self) -> None:
        self._pending_update_job = None
        command = self._build_command()
        self.last_command_var.set(format_bridge_command(command).strip())
        if not self.connected:
            return
        self.worker.submit("update_command", command=command)
        if self.auto_send_var.get() and not self.continuous_send_var.get():
            self.worker.submit("send_once", command=command)
        elif self.continuous_send_var.get():
            self.worker.submit(
                "set_continuous",
                enabled=True,
                command=command,
                send_hz=max(self.send_hz_var.get(), 1.0),
            )

    def refresh_ports(self) -> None:
        ports = list_available_ports()
        self._append_log("Available ports: " + (", ".join(ports) if ports else "<none>"))

    def connect_bridge(self) -> None:
        try:
            config = SerialBridgeConfig(
                port=self.port_var.get().strip(),
                baudrate=int(self.baudrate_var.get().strip()),
                timeout=float(self.timeout_var.get().strip()),
                open_delay_s=float(self.open_delay_var.get().strip()),
            )
        except ValueError as exc:
            messagebox.showerror("Invalid Parameters", str(exc))
            return

        self.port_info_var.set(f"port={config.port} baud={config.baudrate}")
        self.worker.submit("connect", config=config)

    def disconnect_bridge(self) -> None:
        self.continuous_send_var.set(False)
        self.worker.submit("disconnect")

    def send_once(self) -> None:
        if not self._require_connected():
            return
        self.worker.submit("send_once", command=self._build_command())

    def toggle_continuous_send(self) -> None:
        if not self._require_connected():
            self.continuous_send_var.set(False)
            return
        self.worker.submit(
            "set_continuous",
            enabled=self.continuous_send_var.get(),
            command=self._build_command(),
            send_hz=max(self.send_hz_var.get(), 1.0),
        )

    def stop_repeat(self) -> None:
        self.continuous_send_var.set(False)
        if self.connected:
            self.worker.submit(
                "set_continuous",
                enabled=False,
                command=self._build_command(),
                send_hz=max(self.send_hz_var.get(), 1.0),
            )

    def send_relax(self) -> None:
        if not self._require_connected():
            return
        self.continuous_send_var.set(False)
        self.position_var.set(0.0)
        self.kp_var.set(0.0)
        self.velocity_var.set(0.0)
        self.kd_var.set(0.0)
        self.worker.submit("send_relax")

    def _require_connected(self) -> bool:
        if self.connected:
            return True
        messagebox.showwarning("Not Connected", "Connect to the controller board first.")
        return False

    def _build_command(self) -> SerialCommand:
        return SerialCommand(
            position=float(self.position_var.get()),
            kp=float(self.kp_var.get()),
            velocity=float(self.velocity_var.get()),
            kd=float(self.kd_var.get()),
        )

    def _poll_events(self) -> None:
        try:
            while True:
                event_type, payload = self.worker.event_queue.get_nowait()
                self._handle_event(event_type, payload)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _handle_event(self, event_type: str, payload: dict[str, Any]) -> None:
        if event_type == "log":
            self._append_log(payload["message"])
            return
        if event_type == "error":
            self._append_log(f"ERROR: {payload['message']}")
            return
        if event_type == "connected":
            self.connected = True
            self._append_log(f"Connected to {payload['port']} @ {payload['baudrate']} baud.")
            return
        if event_type == "disconnected":
            self.connected = False
            self.last_board_line = ""
            self.last_board_line_var.set("--")
            return
        if event_type == "tx":
            self._append_log(f"TX {payload['line']}")
            return
        if event_type == "serial_line":
            line = payload["line"]
            self.last_board_line = line
            self.last_board_line_var.set(line if line else "<blank line>")
            self._append_log(f"RX {line}")
            return
        if event_type == "stats":
            self.connected = payload["connected"]
            self.tx_count = payload["tx_count"]
            self.rx_count = payload["rx_count"]
            self.last_tx_time = payload["last_tx_time"]
            self.last_rx_time = payload["last_rx_time"]
            self.counter_var.set(f"TX: {self.tx_count}   RX: {self.rx_count}")
            return

    def _append_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _refresh_status(self) -> None:
        now = time.time()
        if not self.connected:
            self._set_status("Disconnected", "#808080", "Connect to the SF_Control serial port.")
        elif self.rx_count > 0 and now - self.last_rx_time < 2.0:
            self._set_status("Running", "#16803C", "Board is replying on serial. Command bridge is active.")
        elif self.tx_count > 0:
            self._set_status(
                "TX Only",
                "#B36B00",
                "Commands are being written. If the motor does not move, confirm the board firmware is running and the port matches PlatformIO monitor.",
            )
        else:
            self._set_status("Connected", "#2D74DA", "Serial bridge is open. Send a command such as 0.500,2.000,0.000,0.100.")
        self.root.after(200, self._refresh_status)

    def _set_status(self, title: str, color: str, detail: str) -> None:
        self.status_var.set(title)
        self.detail_var.set(detail)
        self.status_badge.configure(bg=color)

    def on_close(self) -> None:
        self.worker.stop()
        self.worker.join(timeout=1.0)
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = MotorSerialGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
