"""
MIT CAN motor controller with extra diagnostics.

This script keeps the original MIT protocol layout and adds:
- bus/environment overview on startup
- raw TX/RX logging when --verbose is enabled
- controller statistics on exit
- a diagnose command to verify bus activity and motor response
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

try:
    import can  # type: ignore
except ImportError:
    can = None


MIT_BASE_ID = 0x200
HEARTBEAT_BASE_ID = 0x780
SPECIAL_BASE_ID = 0x000

CMD_CLEAR_ERROR = 0xFB
CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD
CMD_SET_ZERO = 0xFE
CMD_REBOOT = CMD_CLEAR_ERROR  # legacy alias

@dataclass(frozen=True)
class MotorLimits:
    pos_min: float = -12.56
    pos_max: float = 12.56
    vel_min: float = -45.0
    vel_max: float = 45.0
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0
    torque_min: float = -18.0
    torque_max: float = 18.0
    gear_ratio: float = 8.0


@dataclass(frozen=True)
class MITCommand:
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


@dataclass(frozen=True)
class MotorFeedback:
    motor_id: int
    status_code: int
    can_id: int
    position_motor_rad: float
    position_output_rad: float
    velocity_rad_s: float
    torque_nm: float
    mos_temp_c: float
    motor_temp_c: float
    raw: bytes


@dataclass
class ControllerStats:
    tx_count: int = 0
    rx_count: int = 0
    rx_ignored_count: int = 0
    rx_timeout_count: int = 0
    last_tx_id: int | None = None
    last_rx_id: int | None = None
    last_error: str | None = None


def require_python_can() -> None:
    if can is None:
        raise SystemExit(
            "python-can is not installed. Run: pip install python-can\n"
            "If you use a specific USB-CAN device, you may also need its driver/backend."
        )


def clip(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def default_feedback_id(motor_id: int) -> int:
    return HEARTBEAT_BASE_ID + motor_id


def parse_standard_can_id(text: str) -> int:
    value = int(text, 0)
    if not 0 <= value <= 0x7FF:
        raise ValueError(f"CAN ID must be within 0x000..0x7FF, got {value:#x}")
    return value


def parse_optional_standard_can_id(text: str) -> int | None:
    stripped = text.strip()
    if not stripped or stripped.lower() == "auto":
        return None
    return parse_standard_can_id(stripped)


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    value = clip(value, value_min, value_max)
    span = value_max - value_min
    if span <= 0:
        raise ValueError("invalid mapping span")
    return int((value - value_min) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, value_min: float, value_max: float, bits: int) -> float:
    span = value_max - value_min
    if span <= 0:
        raise ValueError("invalid mapping span")
    return value * span / ((1 << bits) - 1) + value_min


def build_special_frame(command_byte: int) -> bytes:
    return bytes([0xFF] * 7 + [command_byte])


def pack_mit_command(command: MITCommand, limits: MotorLimits) -> bytes:
    pos = float_to_uint(command.position, limits.pos_min, limits.pos_max, 16)
    vel = float_to_uint(command.velocity, limits.vel_min, limits.vel_max, 12)
    kp = float_to_uint(command.kp, limits.kp_min, limits.kp_max, 12)
    kd = float_to_uint(command.kd, limits.kd_min, limits.kd_max, 12)
    torque = float_to_uint(command.torque, limits.torque_min, limits.torque_max, 12)

    return bytes(
        [
            (pos >> 8) & 0xFF,
            pos & 0xFF,
            (vel >> 4) & 0xFF,
            ((vel & 0x0F) << 4) | ((kp >> 8) & 0x0F),
            kp & 0xFF,
            (kd >> 4) & 0xFF,
            ((kd & 0x0F) << 4) | ((torque >> 8) & 0x0F),
            torque & 0xFF,
        ]
    )


def parse_feedback(msg: Any, limits: MotorLimits) -> MotorFeedback:
    data = bytes(msg.data)
    if len(data) != 8:
        raise ValueError(f"expected 8-byte feedback, got {len(data)} bytes")

    status_code = data[0]
    motor_id = msg.arbitration_id & 0x7F
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    torque_int = ((data[4] & 0x0F) << 8) | data[5]
    mos_temp = data[6] * 100.0 / 255.0
    motor_temp = data[7] * 100.0 / 255.0

    position_motor = uint_to_float(pos_int, limits.pos_min, limits.pos_max, 16)
    return MotorFeedback(
        motor_id=motor_id,
        status_code=status_code,
        can_id=msg.arbitration_id,
        position_motor_rad=position_motor,
        position_output_rad=position_motor / limits.gear_ratio,
        velocity_rad_s=uint_to_float(vel_int, limits.vel_min, limits.vel_max, 12),
        torque_nm=uint_to_float(torque_int, limits.torque_min, limits.torque_max, 12),
        mos_temp_c=mos_temp,
        motor_temp_c=motor_temp,
        raw=data,
    )


def format_can_frame(msg: Any) -> str:
    data = bytes(msg.data)
    frame_type = "EXT" if msg.is_extended_id else "STD"
    return "id=0x{0:03X} {1} dlc={2} data={3}".format(
        msg.arbitration_id,
        frame_type,
        len(data),
        data.hex(" ") if data else "<empty>",
    )


def print_feedback(feedback: MotorFeedback | None) -> None:
    if feedback is None:
        print("No motor feedback received.")
        return

    print(
        "feedback: motor_id=0x{0:02X} header=0x{1:02X} can_id=0x{2:03X} "
        "pos_motor={3:.4f} rad pos_output={4:.4f} rad vel={5:.4f} rad/s "
        "torque={6:.4f} Nm tmos={7:.1f}C tmotor={8:.1f}C raw={9}".format(
            feedback.motor_id,
            feedback.status_code,
            feedback.can_id,
            feedback.position_motor_rad,
            feedback.position_output_rad,
            feedback.velocity_rad_s,
            feedback.torque_nm,
            feedback.mos_temp_c,
            feedback.motor_temp_c,
            feedback.raw.hex(" "),
        )
    )


def print_bus_overview(bus: Any, args: argparse.Namespace, controller: "MITMotorController | None" = None) -> None:
    print("=== Environment ===")
    print(f"python={sys.version.split()[0]}")
    print(f"python-can={getattr(can, '__version__', 'unknown')}")
    print(f"interface={args.interface} channel={args.channel} bitrate={args.bitrate}")

    if controller is not None:
        print(
            "motor_id=0x{0:02X} mit_id=0x{1:03X} feedback_filter={2} special_id=0x{3:03X}".format(
                controller.motor_id,
                controller.mit_id,
                controller.feedback_id_display,
                controller.special_id,
            )
        )

    channel_info = getattr(bus, "channel_info", None)
    if channel_info:
        print(f"channel_info={channel_info}")

    state = getattr(bus, "state", None)
    if state is not None:
        print(f"bus_state={state}")

    protocol = getattr(bus, "protocol", None)
    if protocol is not None:
        print(f"protocol={protocol}")


def print_controller_stats(stats: ControllerStats) -> None:
    print("=== Controller Stats ===")
    print(
        "tx_count={0} rx_count={1} rx_ignored={2} rx_timeouts={3}".format(
            stats.tx_count,
            stats.rx_count,
            stats.rx_ignored_count,
            stats.rx_timeout_count,
        )
    )
    print(
        "last_tx_id={0} last_rx_id={1}".format(
            "None" if stats.last_tx_id is None else f"0x{stats.last_tx_id:03X}",
            "None" if stats.last_rx_id is None else f"0x{stats.last_rx_id:03X}",
        )
    )
    if stats.last_error:
        print(f"last_error={stats.last_error}")


class MITMotorController:
    def __init__(
        self,
        bus: Any,
        motor_id: int,
        limits: MotorLimits,
        feedback_id: int | None = None,
        verbose: bool = False,
    ):
        self.bus = bus
        self.motor_id = motor_id
        self.limits = limits
        self.feedback_id = feedback_id
        self.feedback_id_hint = default_feedback_id(motor_id)
        self.verbose = verbose
        self.stats = ControllerStats()

    @property
    def mit_id(self) -> int:
        return MIT_BASE_ID + self.motor_id

    @property
    def heartbeat_id(self) -> int:
        return self.feedback_id if self.feedback_id is not None else self.feedback_id_hint

    @property
    def special_id(self) -> int:
        return SPECIAL_BASE_ID + self.motor_id

    @property
    def feedback_id_display(self) -> str:
        if self.feedback_id is not None:
            return f"0x{self.feedback_id:03X}"
        return f"auto (guess 0x{self.feedback_id_hint:03X})"

    def _send(self, arbitration_id: int, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=False,
            data=data,
        )

        try:
            self.bus.send(msg)
            self.stats.tx_count += 1
            self.stats.last_tx_id = arbitration_id
            if self.verbose:
                print(f"TX {format_can_frame(msg)}")
        except Exception as exc:
            self.stats.last_error = str(exc)
            print(f"TX error for id=0x{arbitration_id:03X}: {exc}")
            raise

    def enable(self) -> None:
        self._send(self.special_id, build_special_frame(CMD_ENABLE))

    def disable(self) -> None:
        self._send(self.special_id, build_special_frame(CMD_DISABLE))

    def set_zero(self) -> None:
        self._send(self.special_id, build_special_frame(CMD_SET_ZERO))

    def clear_error(self) -> None:
        self._send(self.special_id, build_special_frame(CMD_CLEAR_ERROR))

    def reboot(self) -> None:
        self.clear_error()

    def send_mit(self, command: MITCommand) -> None:
        self._send(self.mit_id, pack_mit_command(command, self.limits))

    def try_parse_feedback(self, msg: Any) -> MotorFeedback | None:
        if msg.is_extended_id:
            return None
        if (msg.arbitration_id & 0x780) != HEARTBEAT_BASE_ID:
            return None
        try:
            feedback = parse_feedback(msg, self.limits)
        except Exception:
            return None
        if feedback.motor_id != self.motor_id:
            return None
        if self.feedback_id is not None and msg.arbitration_id != self.feedback_id:
            return None
        if self.feedback_id is None:
            self.feedback_id = msg.arbitration_id
        return feedback

    def recv_any(self, timeout: float = 0.1) -> Any | None:
        try:
            msg = self.bus.recv(timeout=timeout)
        except Exception as exc:
            self.stats.last_error = str(exc)
            print(f"RX error: {exc}")
            raise

        if msg is None:
            self.stats.rx_timeout_count += 1
            return None

        self.stats.rx_count += 1
        self.stats.last_rx_id = msg.arbitration_id
        if self.verbose:
            print(f"RX raw {format_can_frame(msg)}")
        return msg

    def recv_feedback(self, timeout: float = 0.05) -> MotorFeedback | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv_any(timeout=max(0.0, deadline - time.time()))
            if msg is None:
                return None
            feedback = self.try_parse_feedback(msg)
            if feedback is None:
                self.stats.rx_ignored_count += 1
                if self.verbose:
                    print(f"RX ignore {format_can_frame(msg)}")
                continue
            if self.verbose:
                print_feedback(feedback)
            return feedback
        self.stats.rx_timeout_count += 1
        return None

    def hold_command(
        self,
        command: MITCommand,
        duration: float,
        send_hz: float = 100.0,
        read_feedback: bool = True,
    ) -> MotorFeedback | None:
        period = 1.0 / send_hz
        end_time = time.time() + duration
        last_feedback = None

        while time.time() < end_time:
            loop_start = time.time()
            self.send_mit(command)
            if read_feedback:
                last_feedback = self.recv_feedback(timeout=min(period * 0.8, 0.03))
            sleep_time = period - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)
        return last_feedback

    def move_between(
        self,
        start: float,
        end: float,
        kp: float,
        kd: float,
        cycles: int,
        hold_s: float,
        send_hz: float,
    ) -> None:
        for cycle in range(cycles):
            if self.verbose:
                print(f"cycle {cycle + 1}/{cycles}: move to {start:.3f} rad")
            self.hold_command(
                MITCommand(position=start, velocity=0.0, kp=kp, kd=kd, torque=0.0),
                duration=hold_s,
                send_hz=send_hz,
            )
            if self.verbose:
                print(f"cycle {cycle + 1}/{cycles}: move to {end:.3f} rad")
            self.hold_command(
                MITCommand(position=end, velocity=0.0, kp=kp, kd=kd, torque=0.0),
                duration=hold_s,
                send_hz=send_hz,
            )


def create_bus(args: argparse.Namespace) -> Any:
    require_python_can()
    kwargs: dict[str, Any] = {
        "interface": args.interface,
        "channel": args.channel,
    }
    if args.interface in {"slcan", "serial"}:
        kwargs["bitrate"] = args.bitrate
        kwargs["tty_baudrate"] = args.tty_baudrate
        kwargs["timeout"] = args.serial_timeout
        kwargs["sleep_after_open"] = args.sleep_after_open
        kwargs["rtscts"] = args.rtscts
    elif args.interface not in {"socketcan", "socketcan_native"}:
        kwargs["bitrate"] = args.bitrate
    return can.Bus(**kwargs)


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--interface", default="socketcan", help="python-can interface, for example socketcan/pcan/slcan")
    parser.add_argument("--channel", default="COM13", help="CAN channel, for example can0/PCAN_USBBUS1/COM13")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate, default 1000000")
    parser.add_argument("--tty-baudrate", type=int, default=115200, help="serial baudrate for slcan/serial adapters")
    parser.add_argument("--serial-timeout", type=float, default=0.001, help="serial timeout for slcan/serial adapters")
    parser.add_argument("--sleep-after-open", type=float, default=2.0, help="delay after opening an slcan/serial adapter")
    parser.add_argument("--rtscts", action="store_true", help="enable RTS/CTS for slcan/serial adapters")
    parser.add_argument("--motor-id", type=parse_standard_can_id, default=0x01, help="motor receive CAN ID, default 0x01")
    parser.add_argument(
        "--feedback-id",
        type=parse_optional_standard_can_id,
        default=None,
        help="motor feedback CAN ID, for example 0x781; use 'auto' to use 0x780 + motor_id",
    )
    parser.add_argument("--timeout", type=float, default=0.1, help="receive timeout in seconds")
    parser.add_argument("--verbose", action="store_true", help="print raw TX/RX frames and parsed feedback")

    parser.add_argument("--pos-min", type=float, default=-25.12, help="MIT position minimum")
    parser.add_argument("--pos-max", type=float, default=25.12, help="MIT position maximum")
    parser.add_argument("--vel-min", type=float, default=-45.0, help="MIT velocity minimum")
    parser.add_argument("--vel-max", type=float, default=45.0, help="MIT velocity maximum")
    parser.add_argument("--kp-min", type=float, default=0.0, help="MIT kp minimum")
    parser.add_argument("--kp-max", type=float, default=500.0, help="MIT kp maximum")
    parser.add_argument("--kd-min", type=float, default=0.0, help="MIT kd minimum")
    parser.add_argument("--kd-max", type=float, default=5.0, help="MIT kd maximum")
    parser.add_argument("--torque-min", type=float, default=-18.0, help="MIT torque minimum")
    parser.add_argument("--torque-max", type=float, default=18.0, help="MIT torque maximum")
    parser.add_argument("--gear-ratio", type=float, default=8.0, help="gear ratio for output position conversion")


def build_limits(args: argparse.Namespace) -> MotorLimits:
    return MotorLimits(
        pos_min=args.pos_min,
        pos_max=args.pos_max,
        vel_min=args.vel_min,
        vel_max=args.vel_max,
        kp_min=args.kp_min,
        kp_max=args.kp_max,
        kd_min=args.kd_min,
        kd_max=args.kd_max,
        torque_min=args.torque_min,
        torque_max=args.torque_max,
        gear_ratio=args.gear_ratio,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control an MIT-mode motor over CAN")
    add_common_arguments(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("enable", help="send enable command")
    subparsers.add_parser("disable", help="send disable command")
    subparsers.add_parser("zero", help="set current position as zero")
    subparsers.add_parser("clear-error", help="clear motor fault state")
    subparsers.add_parser("reboot", help="legacy alias for clear-error")

    monitor = subparsers.add_parser("monitor", help="listen for motor feedback")
    monitor.add_argument("--count", type=int, default=10, help="number of feedback frames to collect")

    move = subparsers.add_parser("move", help="send MIT command and hold it for some time")
    move.add_argument("--position", type=float, default=0.0, help="target position in rad")
    move.add_argument("--velocity", type=float, default=0.0, help="target velocity in rad/s")
    move.add_argument("--kp", type=float, default=20.0, help="position gain")
    move.add_argument("--kd", type=float, default=1.0, help="velocity gain")
    move.add_argument("--torque", type=float, default=0.0, help="feedforward torque in Nm")
    move.add_argument("--duration", type=float, default=2.0, help="hold time in seconds")
    move.add_argument("--send-hz", type=float, default=100.0, help="command send rate in Hz")
    move.add_argument("--no-enable", action="store_true", help="skip enable before move")

    sweep = subparsers.add_parser("sweep", help="move between two positions")
    sweep.add_argument("--start", type=float, default=-1.0, help="start position in rad")
    sweep.add_argument("--end", type=float, default=1.0, help="end position in rad")
    sweep.add_argument("--kp", type=float, default=20.0, help="position gain")
    sweep.add_argument("--kd", type=float, default=1.0, help="velocity gain")
    sweep.add_argument("--cycles", type=int, default=3, help="number of cycles")
    sweep.add_argument("--hold", type=float, default=1.0, help="hold time at each point")
    sweep.add_argument("--send-hz", type=float, default=100.0, help="command send rate in Hz")

    demo = subparsers.add_parser("demo", help="simple enable -> optional zero -> sweep -> disable demo")
    demo.add_argument("--amplitude", type=float, default=1.0, help="sweep amplitude in rad")
    demo.add_argument("--kp", type=float, default=20.0, help="position gain")
    demo.add_argument("--kd", type=float, default=1.0, help="velocity gain")
    demo.add_argument("--cycles", type=int, default=2, help="number of cycles")
    demo.add_argument("--hold", type=float, default=1.0, help="hold time at each point")
    demo.add_argument("--send-hz", type=float, default=100.0, help="command send rate in Hz")
    demo.add_argument("--set-zero-first", action="store_true", help="set current position to zero before demo")

    diagnose = subparsers.add_parser("diagnose", help="run bus and motor diagnostics")
    diagnose.add_argument("--listen-seconds", type=float, default=2.0, help="passive listen time before sending")
    diagnose.add_argument("--post-send-seconds", type=float, default=1.0, help="passive listen time after sending")
    diagnose.add_argument("--send-hz", type=float, default=20.0, help="send rate for MIT diagnostic frames")
    diagnose.add_argument("--test-position", type=float, default=0.0, help="test position for MIT diagnostic frames")
    diagnose.add_argument("--test-velocity", type=float, default=0.0, help="test velocity for MIT diagnostic frames")
    diagnose.add_argument("--test-kp", type=float, default=5.0, help="test kp for MIT diagnostic frames")
    diagnose.add_argument("--test-kd", type=float, default=0.2, help="test kd for MIT diagnostic frames")
    diagnose.add_argument("--test-torque", type=float, default=0.0, help="test torque for MIT diagnostic frames")
    diagnose.add_argument("--no-enable", action="store_true", help="skip enable before MIT diagnostic frames")

    return parser


def passive_listen(controller: MITMotorController, duration: float, label: str) -> int:
    print(f"=== {label} ({duration:.2f}s) ===")
    deadline = time.time() + duration
    captured = 0

    while time.time() < deadline:
        msg = controller.recv_any(timeout=min(0.2, max(0.0, deadline - time.time())))
        if msg is None:
            continue
        captured += 1
        print(f"RX {captured}: {format_can_frame(msg)}")
        feedback = controller.try_parse_feedback(msg)
        if feedback is not None:
            print_feedback(feedback)

    if captured == 0:
        print("No CAN frames captured in this stage.")
    return captured


def run_diagnose(controller: MITMotorController, args: argparse.Namespace) -> int:
    print("=== Diagnostic Start ===")
    print("Goal: verify bus open, bus receive, bus transmit, and motor heartbeat response.")

    captured_before = passive_listen(controller, args.listen_seconds, "Passive Listen Before Send")

    if not args.no_enable:
        print("Sending enable frame...")
        controller.enable()
        time.sleep(0.1)
    else:
        print("Skipping enable frame because --no-enable was set.")

    command = MITCommand(
        position=args.test_position,
        velocity=args.test_velocity,
        kp=args.test_kp,
        kd=args.test_kd,
        torque=args.test_torque,
    )
    print(
        "Sending MIT test command: pos={0:.3f} vel={1:.3f} kp={2:.3f} kd={3:.3f} torque={4:.3f}".format(
            command.position,
            command.velocity,
            command.kp,
            command.kd,
            command.torque,
        )
    )

    feedback = controller.hold_command(
        command,
        duration=max(args.post_send_seconds, 0.1),
        send_hz=max(args.send_hz, 1.0),
        read_feedback=True,
    )
    if feedback is not None:
        print("Heartbeat feedback received during MIT command stage:")
        print_feedback(feedback)
    else:
        print("No heartbeat feedback received during MIT command stage.")

    captured_after = passive_listen(controller, args.post_send_seconds, "Passive Listen After Send")

    print("=== Diagnostic Summary ===")
    print(
        "captured_before_send={0} captured_after_send={1} feedback_received={2}".format(
            captured_before,
            captured_after,
            feedback is not None,
        )
    )
    print(
        "If tx_count increases but rx_count stays 0, check CAN-H/CAN-L wiring, bitrate, bus termination, channel selection, and adapter driver."
    )
    print(
        "If other frames are visible but motor feedback {0} never appears, check motor power, motor ID, feedback ID, enable sequence, and protocol match.".format(
            controller.feedback_id_display
        )
    )
    return 0


def run_command(args: argparse.Namespace) -> int:
    limits = build_limits(args)
    bus = create_bus(args)
    controller = MITMotorController(
        bus,
        args.motor_id,
        limits,
        feedback_id=args.feedback_id,
        verbose=args.verbose,
    )

    try:
        print_bus_overview(bus, args, controller)

        if args.command == "enable":
            controller.enable()
            print(f"Enable command sent to motor 0x{args.motor_id:02X}.")

        elif args.command == "disable":
            controller.disable()
            print(f"Disable command sent to motor 0x{args.motor_id:02X}.")

        elif args.command == "zero":
            controller.set_zero()
            print(f"Zero command sent to motor 0x{args.motor_id:02X}.")

        elif args.command in {"clear-error", "reboot"}:
            controller.clear_error()
            print(f"Clear-error command sent to motor 0x{args.motor_id:02X}.")

        elif args.command == "monitor":
            print(f"Listening for feedback {controller.feedback_id_display} ...")
            received = 0
            while received < args.count:
                feedback = controller.recv_feedback(timeout=args.timeout)
                if feedback is None:
                    print("Feedback wait timed out.")
                    continue
                print_feedback(feedback)
                received += 1

        elif args.command == "move":
            if not args.no_enable:
                print("Sending enable before move...")
                controller.enable()
                time.sleep(0.05)

            command = MITCommand(
                position=args.position,
                velocity=args.velocity,
                kp=args.kp,
                kd=args.kd,
                torque=args.torque,
            )
            print(
                "Move command: pos={0:.3f} vel={1:.3f} kp={2:.3f} kd={3:.3f} torque={4:.3f} duration={5:.3f}s send_hz={6:.1f}".format(
                    command.position,
                    command.velocity,
                    command.kp,
                    command.kd,
                    command.torque,
                    args.duration,
                    args.send_hz,
                )
            )
            feedback = controller.hold_command(
                command,
                duration=args.duration,
                send_hz=args.send_hz,
                read_feedback=True,
            )
            print_feedback(feedback)

        elif args.command == "sweep":
            print("Sending enable before sweep...")
            controller.enable()
            time.sleep(0.05)
            print(
                "Sweep command: start={0:.3f} end={1:.3f} cycles={2} hold={3:.3f}s send_hz={4:.1f}".format(
                    args.start,
                    args.end,
                    args.cycles,
                    args.hold,
                    args.send_hz,
                )
            )
            controller.move_between(
                start=args.start,
                end=args.end,
                kp=args.kp,
                kd=args.kd,
                cycles=args.cycles,
                hold_s=args.hold,
                send_hz=args.send_hz,
            )
            print("Sweep complete.")

        elif args.command == "demo":
            print("Sending enable before demo...")
            controller.enable()
            time.sleep(0.05)
            if args.set_zero_first:
                print("Sending zero command before demo...")
                controller.set_zero()
                time.sleep(0.1)
            print(
                "Demo command: amplitude={0:.3f} cycles={1} hold={2:.3f}s send_hz={3:.1f}".format(
                    args.amplitude,
                    args.cycles,
                    args.hold,
                    args.send_hz,
                )
            )
            controller.move_between(
                start=-abs(args.amplitude),
                end=abs(args.amplitude),
                kp=args.kp,
                kd=args.kd,
                cycles=args.cycles,
                hold_s=args.hold,
                send_hz=args.send_hz,
            )
            controller.disable()
            print("Demo complete.")

        elif args.command == "diagnose":
            return run_diagnose(controller, args)

        else:
            raise ValueError(f"unsupported command: {args.command}")

        return 0
    finally:
        print_controller_stats(controller.stats)
        shutdown = getattr(bus, "shutdown", None)
        if callable(shutdown):
            shutdown()


def self_check() -> None:
    limits = MotorLimits()
    packed = pack_mit_command(
        MITCommand(position=0.0, velocity=10.0, kp=0.0, kd=1.0, torque=0.0),
        limits,
    )
    expected = bytes([0x7F, 0xFF, 0x9C, 0x60, 0x00, 0x33, 0x37, 0xFF])
    if packed != expected:
        raise AssertionError(f"MIT pack mismatch: got {packed.hex()} expected {expected.hex()}")

    sample_feedback = parse_feedback(
        SimpleNamespace(
            arbitration_id=0x781,
            data=bytes([0x00, 0x7F, 0xFF, 0x80, 0x03, 0xFF, 0x80, 0x40]),
        ),
        limits,
    )
    if sample_feedback.motor_id != 0x01 or sample_feedback.status_code != 0x00:
        raise AssertionError(
            "feedback parse mismatch: expected motor_id=0x01 status=0x00, "
            f"got motor_id=0x{sample_feedback.motor_id:02X} status=0x{sample_feedback.status_code:02X}"
        )


def main() -> int:
    self_check()
    parser = build_parser()
    args = parser.parse_args()
    return run_command(args)


if __name__ == "__main__":
    sys.exit(main())
