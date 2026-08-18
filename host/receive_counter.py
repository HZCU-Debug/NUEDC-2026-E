"""接收 ESP32 通信 Demo 的计数消息，并显示交付模式和计数值"""

import argparse
import time

from link import Delivery, EventType, Link

COUNTER_MESSAGE = 1


def decode_counter(message_type: int, payload: bytes) -> int | None:
    if message_type != COUNTER_MESSAGE or len(payload) != 4:
        return None
    return int.from_bytes(payload, "big")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive reliable or unreliable counter messages from ESP32"
    )
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    import serial

    connection = serial.Serial(args.port, args.baud, timeout=0)
    link = Link(connection, capacity=4, retry_interval_ms=50)
    try:
        while True:
            event = link.poll()
            if event.type == EventType.MESSAGE:
                counter = decode_counter(event.message.type, event.message.payload)
                if counter is not None:
                    mode = (
                        "reliable"
                        if event.message.delivery == Delivery.RELIABLE
                        else "unreliable"
                    )
                    print(f"{mode}: {counter}")
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()


if __name__ == "__main__":
    main()
