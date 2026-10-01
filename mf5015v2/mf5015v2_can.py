import argparse
import struct
import sys
import time
from dataclasses import dataclass


CMD_READ_MULTI_TURN_ANGLE = 0x92
CMD_READ_SINGLE_TURN_ANGLE = 0x94
CMD_READ_STATUS_1 = 0x9A
CMD_CLEAR_ERROR = 0x9B
CMD_READ_STATUS_2 = 0x9C
CMD_READ_STATUS_3 = 0x9D
CMD_MOTOR_OFF = 0x80
CMD_MOTOR_STOP = 0x81
CMD_MOTOR_RUNNING = 0x88
CMD_TORQUE_CLOSED_LOOP = 0xA1
CMD_SPEED_CLOSED_LOOP = 0xA2
CMD_MULTI_TURN_POSITION = 0xA3
CMD_MULTI_TURN_POSITION_SPEED = 0xA4

SEND_BASE_ID = 0x140
REPLY_BASE_ID = 0x240


def import_can():
    try:
        import can
    except ModuleNotFoundError:
        print("python-can is not installed.")
        print(f"Install it with: {sys.executable} -m pip install python-can")
        raise SystemExit(1)
    return can


def int16_le(data, offset):
    return struct.unpack_from("<h", bytes(data), offset)[0]


def uint16_le(data, offset):
    return struct.unpack_from("<H", bytes(data), offset)[0]


def int32_le(data, offset):
    return struct.unpack_from("<i", bytes(data), offset)[0]


def int56_le(data, offset):
    raw = int.from_bytes(bytes(data[offset : offset + 7]), "little")
    if raw & (1 << 55):
        raw -= 1 << 56
    return raw


def clamp_int16(value, name):
    if value < -32768 or value > 32767:
        raise ValueError(f"{name} is outside int16 range: {value}")
    return value


def clamp_int32(value, name):
    if value < -2147483648 or value > 2147483647:
        raise ValueError(f"{name} is outside int32 range: {value}")
    return value


@dataclass
class MotorFeedback:
    command: int
    temperature_c: int
    iq_raw: int
    speed_dps: int
    angle_or_encoder_raw: int

    @property
    def iq_a_v4_scale(self):
        return self.iq_raw * 0.01


class Mf5015Can:
    def __init__(self, bus, motor_id=1, timeout=0.5):
        if motor_id < 1 or motor_id > 32:
            raise ValueError("motor_id must be in the common 1..32 range")
        self.bus = bus
        self.motor_id = motor_id
        self.timeout = timeout
        self.can = import_can()

    @property
    def tx_id(self):
        return SEND_BASE_ID + self.motor_id

    @property
    def reply_ids(self):
        return {REPLY_BASE_ID + self.motor_id, SEND_BASE_ID + self.motor_id}

    def send(self, command, payload=b"\x00" * 7, expect_reply=True):
        data = bytes([command]) + bytes(payload)
        if len(data) != 8:
            raise ValueError("MF/RMD CAN commands use exactly 8 data bytes")

        msg = self.can.Message(
            arbitration_id=self.tx_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)
        if not expect_reply:
            return None
        return self.recv_reply(command)

    def recv_reply(self, command):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            msg = self.bus.recv(timeout=remaining)
            if msg is None:
                break
            if msg.is_extended_id:
                continue
            if msg.arbitration_id not in self.reply_ids:
                continue
            if len(msg.data) >= 1 and msg.data[0] == command:
                return msg
        raise TimeoutError(
            f"no reply for command 0x{command:02X}; check CAN wiring, baudrate, and motor ID"
        )

    def motor_off(self):
        return self.send(CMD_MOTOR_OFF)

    def stop(self):
        return self.send(CMD_MOTOR_STOP)

    def run(self):
        return self.send(CMD_MOTOR_RUNNING)

    def clear_error(self):
        return self.send(CMD_CLEAR_ERROR)

    def read_status_1(self):
        return self.send(CMD_READ_STATUS_1)

    def read_status_2(self):
        msg = self.send(CMD_READ_STATUS_2)
        return parse_feedback(msg.data)

    def read_status_3(self):
        return self.send(CMD_READ_STATUS_3)

    def read_multi_turn_angle_deg(self):
        msg = self.send(CMD_READ_MULTI_TURN_ANGLE)
        raw = int56_le(msg.data, 1)
        return raw * 0.01

    def read_single_turn_angle_deg(self):
        msg = self.send(CMD_READ_SINGLE_TURN_ANGLE)
        raw = uint16_le(msg.data, 6)
        return raw * 0.01

    def torque_current(self, amps):
        iq_control = clamp_int16(round(amps / 0.01), "amps as 0.01A/LSB")
        payload = b"\x00\x00\x00" + struct.pack("<h", iq_control) + b"\x00\x00"
        msg = self.send(CMD_TORQUE_CLOSED_LOOP, payload)
        return parse_feedback(msg.data)

    def speed(self, dps):
        speed_control = clamp_int32(round(dps / 0.01), "dps as 0.01dps/LSB")
        payload = b"\x00\x00\x00" + struct.pack("<i", speed_control)
        msg = self.send(CMD_SPEED_CLOSED_LOOP, payload)
        return parse_feedback(msg.data)

    def multi_turn_position(self, degrees):
        angle_control = clamp_int32(round(degrees / 0.01), "degrees as 0.01deg/LSB")
        payload = b"\x00\x00\x00" + struct.pack("<i", angle_control)
        msg = self.send(CMD_MULTI_TURN_POSITION, payload)
        return parse_feedback(msg.data)

    def multi_turn_position_speed(self, degrees, max_speed_dps):
        if max_speed_dps < 0 or max_speed_dps > 65535:
            raise ValueError("max_speed_dps must be 0..65535")
        angle_control = clamp_int32(round(degrees / 0.01), "degrees as 0.01deg/LSB")
        payload = b"\x00" + struct.pack("<H", round(max_speed_dps)) + struct.pack(
            "<i", angle_control
        )
        msg = self.send(CMD_MULTI_TURN_POSITION_SPEED, payload)
        return parse_feedback(msg.data)


def parse_feedback(data):
    if len(data) != 8:
        raise ValueError("feedback frame must contain 8 bytes")
    return MotorFeedback(
        command=data[0],
        temperature_c=struct.unpack("b", bytes([data[1]]))[0],
        iq_raw=int16_le(data, 2),
        speed_dps=int16_le(data, 4),
        angle_or_encoder_raw=int16_le(data, 6),
    )


def hex_frame(msg):
    data = " ".join(f"{b:02X}" for b in msg.data)
    return f"id=0x{msg.arbitration_id:X} data=[{data}]"


def print_feedback(feedback):
    print(f"command:       0x{feedback.command:02X}")
    print(f"temperature:   {feedback.temperature_c} C")
    print(f"iq raw:        {feedback.iq_raw}")
    print(f"iq approx:     {feedback.iq_a_v4_scale:.2f} A (0.01A/LSB protocol scale)")
    print(f"speed:         {feedback.speed_dps} dps")
    print(f"angle/encoder: {feedback.angle_or_encoder_raw}")


def open_bus(args):
    can = import_can()
    kwargs = {
        "interface": args.interface,
        "channel": args.channel,
    }
    if args.bitrate:
        kwargs["bitrate"] = args.bitrate
    return can.Bus(**kwargs)


def require_abs_limit(value, limit, name):
    if abs(value) > limit:
        raise SystemExit(
            f"{name}={value} exceeds safety limit {limit}. "
            f"Raise --safety-current-a or --safety-speed-dps only after bench testing."
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description="MF5015-V2 / MF-RMD style CAN control helper using python-can."
    )
    parser.add_argument("--interface", default="slcan", help="python-can interface")
    parser.add_argument("--channel", default="COM3", help="CAN channel, e.g. COM3/can0/PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate")
    parser.add_argument("--motor-id", type=int, default=1, help="motor ID, usually 1..32")
    parser.add_argument("--timeout", type=float, default=0.5, help="reply timeout seconds")
    parser.add_argument("--safety-current-a", type=float, default=0.5)
    parser.add_argument("--safety-speed-dps", type=float, default=180.0)

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="read status and angles")
    sub.add_parser("off", help="motor off, no torque output")
    sub.add_parser("stop", help="stop movement but keep driver powered")
    sub.add_parser("run", help="resume motor running state")
    sub.add_parser("clear-error", help="clear driver error flag")

    torque = sub.add_parser("torque", help="torque/current closed-loop command")
    torque.add_argument("--amps", type=float, required=True)
    torque.add_argument("--duration", type=float, default=0.2)

    speed = sub.add_parser("speed", help="speed closed-loop command")
    speed.add_argument("--dps", type=float, required=True, help="degrees per second")
    speed.add_argument("--duration", type=float, default=1.0)

    pos = sub.add_parser("position", help="multi-turn position command")
    pos.add_argument("--degrees", type=float, required=True)
    pos.add_argument("--max-speed-dps", type=float, default=60.0)

    return parser


def main():
    args = build_parser().parse_args()
    with open_bus(args) as bus:
        motor = Mf5015Can(bus, motor_id=args.motor_id, timeout=args.timeout)

        if args.command == "status":
            status1 = motor.read_status_1()
            status2 = motor.read_status_2()
            status3 = motor.read_status_3()
            print("status1 raw:", hex_frame(status1))
            print_feedback(status2)
            print("status3 raw:", hex_frame(status3))
            print(f"multi-turn angle:  {motor.read_multi_turn_angle_deg():.2f} deg")
            print(f"single-turn angle: {motor.read_single_turn_angle_deg():.2f} deg")
            return

        if args.command == "off":
            print(hex_frame(motor.motor_off()))
            return

        if args.command == "stop":
            print(hex_frame(motor.stop()))
            return

        if args.command == "run":
            print(hex_frame(motor.run()))
            return

        if args.command == "clear-error":
            print(hex_frame(motor.clear_error()))
            return

        if args.command == "torque":
            require_abs_limit(args.amps, args.safety_current_a, "amps")
            feedback = motor.torque_current(args.amps)
            print_feedback(feedback)
            if args.duration > 0:
                time.sleep(args.duration)
                print("stopping after duration")
                print(hex_frame(motor.stop()))
            return

        if args.command == "speed":
            require_abs_limit(args.dps, args.safety_speed_dps, "dps")
            feedback = motor.speed(args.dps)
            print_feedback(feedback)
            if args.duration > 0:
                time.sleep(args.duration)
                print("stopping after duration")
                print(hex_frame(motor.stop()))
            return

        if args.command == "position":
            require_abs_limit(args.max_speed_dps, args.safety_speed_dps, "max_speed_dps")
            feedback = motor.multi_turn_position_speed(args.degrees, args.max_speed_dps)
            print_feedback(feedback)
            return


if __name__ == "__main__":
    main()
