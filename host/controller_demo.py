import argparse
import struct
import time

from link import Delivery, Link, SendResult

VELOCITY_LIMIT = 1000
VELOCITY_MESSAGE = 1


def axis_to_velocity(value: float, deadzone: float, invert: bool = False) -> int:
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone:
        return 0
    if invert:
        value = -value
    return round(value * VELOCITY_LIMIT)


def velocity_payload(velocity: int) -> bytes:
    if not -VELOCITY_LIMIT <= velocity <= VELOCITY_LIMIT:
        raise ValueError("velocity must be between -1000 and 1000")
    return struct.pack(">h", velocity)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a game controller axis to the ESP32 motor demo"
    )
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--axis", type=int, default=3, help="pygame axis index")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--deadzone", type=float, default=0.1, help="joystick deadzone")
    parser.add_argument("--invert", action="store_true", help="invert axis direction")
    args = parser.parse_args()
    if args.axis < 0:
        parser.error("axis must be non-negative")
    if not 0.0 <= args.deadzone < 1.0:
        parser.error("deadzone must be between 0 and 1")
    return args


def main() -> None:
    args = parse_args()

    import pygame
    import serial

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("no game controller found")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    if args.axis >= joystick.get_numaxes():
        raise SystemExit(
            f"axis {args.axis} is unavailable, controller has {joystick.get_numaxes()} axes"
        )

    connection = serial.Serial(args.port, args.baud, timeout=0.1)
    link = Link(connection, capacity=2, retry_interval_ms=50)
    last_velocity = None
    try:
        while True:
            pygame.event.pump()
            velocity = axis_to_velocity(
                joystick.get_axis(args.axis), args.deadzone, args.invert
            )
            result = link.send(
                VELOCITY_MESSAGE, velocity_payload(velocity), Delivery.UNRELIABLE
            )
            if result != SendResult.ACCEPTED:
                raise RuntimeError(f"failed to send velocity: {result}")
            link.poll()
            if velocity != last_velocity:
                print(f"velocity: {velocity}")
                last_velocity = velocity
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            link.send(VELOCITY_MESSAGE, velocity_payload(0), Delivery.UNRELIABLE)
        except serial.SerialException:
            pass
        connection.close()
        joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()
