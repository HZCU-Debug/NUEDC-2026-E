"""发送固定拼图 Demo 数据"""

import argparse
import struct
import time

from link import Delivery, EventType, Link, SendResult


PIECE_COUNT_MESSAGE = 0x01
PIECE_DATA_MESSAGE = 0x02
MODE_REQUEST_MESSAGE = 0x03
LINK_CAPACITY = 16
PIECE_FORMAT = ">Bhhhhh"

DEMO_PIECES = (
    (1, 180, 110, 60, 250, -1234),
    (2, 90, 120, 205, 255, 4567),
    (3, 155, 130, 80, 200, -9000),
    (4, 100, 140, 190, 195, 18000),
)


def build_demo_messages() -> list[tuple[int, bytes]]:
    """生成固定拼图 Demo 消息"""
    messages = [(PIECE_COUNT_MESSAGE, bytes((len(DEMO_PIECES),)))]
    messages.extend(
        (PIECE_DATA_MESSAGE, struct.pack(PIECE_FORMAT, *piece))
        for piece in DEMO_PIECES
    )
    return messages


class ReliablePuzzleSender:
    """逐条发送可靠消息并等待 ESP32 确认"""

    def __init__(self, link, messages: list[tuple[int, bytes]]) -> None:
        self._link = link
        self._messages = tuple(messages)
        self._index = 0
        self._waiting = False
        self.delivered_count = 0

    @property
    def complete(self) -> bool:
        """全部消息已确认时返回 True"""
        return self._index >= len(self._messages) and not self._waiting

    def step(self, event=None) -> None:
        """轮询一次链路并发送下一条可发送的消息"""
        if event is None:
            event = self._link.poll()
        if event.type == EventType.DELIVERED and self._waiting:
            self._waiting = False
            self._index += 1
            self.delivered_count += 1

        if self._waiting or self._index >= len(self._messages):
            return

        message_type, payload = self._messages[self._index]
        result = self._link.send(message_type, payload, Delivery.RELIABLE)
        if result == SendResult.ACCEPTED:
            self._waiting = True
        elif result != SendResult.BUSY:
            raise RuntimeError(f"failed to send message: result={result}")


def parse_mode_request(event) -> int | None:
    """解析 ESP32 可靠发送的视觉模式"""
    if (
        event.type != EventType.MESSAGE
        or event.message.type != MODE_REQUEST_MESSAGE
        or event.message.delivery != Delivery.RELIABLE
        or len(event.message.payload) != 1
    ):
        return None
    mode = event.message.payload[0]
    return mode if mode in (0, 1, 2, 3) else None


class PuzzleController:
    """处理视觉标定模式并可靠发送固定拼图结果"""

    def __init__(self, link) -> None:
        self._link = link
        self.sender: ReliablePuzzleSender | None = None
        self.calibration_active = False

    def step(self) -> int | None:
        """轮询一次链路，返回本轮接受的题号"""
        event = self._link.poll()
        mode = parse_mode_request(event)
        accepted_mode = None
        if mode is not None and (self.sender is None or self.sender.complete):
            if mode == 0:
                self.calibration_active = not self.calibration_active
                accepted_mode = mode
            elif not self.calibration_active:
                self.sender = ReliablePuzzleSender(
                    self._link, build_demo_messages()
                )
                accepted_mode = mode
        if self.sender is not None:
            self.sender.step(event)
        return accepted_mode


def parse_args() -> argparse.Namespace:
    """解析串口参数"""
    parser = argparse.ArgumentParser(description="Simulate Raspberry Pi puzzle output")
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    return parser.parse_args()


def main() -> None:
    """发送模拟拼图结果直到全部收到确认"""
    args = parse_args()

    import serial

    connection = serial.Serial(args.port, args.baud, timeout=0)
    link = Link(connection, capacity=LINK_CAPACITY, retry_interval_ms=50)
    controller = PuzzleController(link)
    last_delivered = 0
    message_count = len(build_demo_messages())
    print(f"waiting for puzzle mode on {args.port} at {args.baud} baud")
    try:
        while True:
            mode = controller.step()
            if mode is not None:
                last_delivered = 0
                print(f"mode {mode} accepted", flush=True)
            sender = controller.sender
            if sender is not None and sender.delivered_count != last_delivered:
                last_delivered = sender.delivered_count
                print(
                    f"delivered {last_delivered}/{message_count}",
                    flush=True,
                )
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("cancelled")
    finally:
        link.cancel()
        connection.close()


if __name__ == "__main__":
    main()
