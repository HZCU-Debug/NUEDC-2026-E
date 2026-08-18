"""向 ESP32 发送单轴电机速度或绝对位置命令"""

import argparse
import math
import struct
import time

from link import Delivery, EventType, Link, SendResult


SPEED_MESSAGE = 0x01
POSITION_MESSAGE = 0x02
AXES = frozenset("XYZR")
MAXIMUM_RPM = 6000


def speed_payload(axis: str, rpm: int) -> bytes:
    """编码单轴带符号 RPM 命令"""
    normalized_axis = axis.upper()
    if normalized_axis not in AXES:
        raise ValueError("axis must be X, Y, Z, or R")
    if not -MAXIMUM_RPM <= rpm <= MAXIMUM_RPM:
        raise ValueError("rpm must be between -6000 and 6000")
    return normalized_axis.encode("ascii") + struct.pack(">h", rpm)


def position_payload(axis: str, degrees: float, rpm: int) -> bytes:
    """编码单轴绝对角度和最大 RPM 命令"""
    normalized_axis = axis.upper()
    if normalized_axis not in AXES:
        raise ValueError("axis must be X, Y, Z, or R")
    if not math.isfinite(degrees):
        raise ValueError("degrees must be finite")
    if not 1 <= rpm <= MAXIMUM_RPM:
        raise ValueError("rpm must be between 1 and 6000")
    return normalized_axis.encode("ascii") + struct.pack(">fH", degrees, rpm)


def parse_args() -> argparse.Namespace:
    """解析串口参数"""
    parser = argparse.ArgumentParser(description="Test one ESP32 motor at a time")
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    return parser.parse_args()


def send_speed(link: Link, axis: str, rpm: int) -> None:
    """可靠发送单轴速度并等待 ESP32 确认"""
    result = link.send(
        SPEED_MESSAGE, speed_payload(axis, rpm), Delivery.RELIABLE
    )
    if result != SendResult.ACCEPTED:
        raise RuntimeError(f"failed to send speed: result={result}")
    while link.poll().type != EventType.DELIVERED:
        time.sleep(0.001)


def send_position(link: Link, axis: str, degrees: float, rpm: int) -> None:
    """可靠发送单轴绝对位置并等待 ESP32 确认"""
    result = link.send(
        POSITION_MESSAGE,
        position_payload(axis, degrees, rpm),
        Delivery.RELIABLE,
    )
    if result != SendResult.ACCEPTED:
        raise RuntimeError(f"failed to send position: result={result}")
    while link.poll().type != EventType.DELIVERED:
        time.sleep(0.001)


def main() -> None:
    """交互读取单轴速度或绝对位置命令并逐条发送"""
    args = parse_args()

    import serial

    connection = serial.Serial(args.port, args.baud, timeout=0)
    link = Link(connection, capacity=7, retry_interval_ms=50)
    print("speed: X 300; position: X 90 300; use X 0 to stop; q to quit")
    try:
        while True:
            command = input("motor> ").strip()
            if command.lower() in {"q", "quit", "exit"}:
                break
            try:
                fields = command.split()
                if len(fields) == 2:
                    axis, rpm_text = fields
                    rpm = int(rpm_text)
                    send_speed(link, axis, rpm)
                    print(f"delivered {axis.upper()}={rpm} RPM")
                elif len(fields) == 3:
                    axis, degrees_text, rpm_text = fields
                    degrees = float(degrees_text)
                    rpm = int(rpm_text)
                    send_position(link, axis, degrees, rpm)
                    print(
                        f"delivered {axis.upper()}={degrees:g} degrees "
                        f"at {rpm} RPM"
                    )
                else:
                    raise ValueError("use AXIS RPM or AXIS DEGREES RPM")
            except ValueError as error:
                print(error)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        link.cancel()
        connection.close()


if __name__ == "__main__":
    main()
