import time

from link import Delivery, EventType, Link, SendResult


class FakeSerial:
    def __init__(self) -> None:
        self.received = bytearray()
        self.transmitted = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self.received)

    def read(self, size: int = 1) -> bytes:
        data = bytes(self.received[:size])
        del self.received[:size]
        return data

    def write(self, data: bytes) -> int:
        self.transmitted.extend(data)
        return len(data)

    def receive(self, data: bytes) -> None:
        self.received.extend(data)


def test_link_protocol() -> None:
    serial = FakeSerial()
    link = Link(serial, capacity=8, retry_interval_ms=50)

    assert link.send(0, b"", Delivery.UNRELIABLE) == SendResult.INVALID_ARGUMENT
    assert link.send(1, 3, Delivery.UNRELIABLE) == SendResult.INVALID_ARGUMENT
    assert link.send(1, b"", 2) == SendResult.INVALID_ARGUMENT
    assert (
        link.send(1, bytes(9), Delivery.UNRELIABLE)
        == SendResult.PAYLOAD_TOO_LARGE
    )

    assert (
        link.send(1, b"\x11\x00\x22", Delivery.UNRELIABLE)
        == SendResult.ACCEPTED
    )
    assert serial.transmitted == bytes.fromhex("03 01 11 03 22 31 00")

    serial.transmitted.clear()
    assert link.send(1, b"\x33", Delivery.RELIABLE) == SendResult.ACCEPTED
    assert serial.transmitted == bytes.fromhex("02 81 03 33 f9 00")
    assert link.send(2, b"", Delivery.UNRELIABLE) == SendResult.BUSY
    assert link.send(2, b"", Delivery.RELIABLE) == SendResult.BUSY

    serial.receive(bytes.fromhex("01 01 01 01 00"))
    assert link.poll().type == EventType.DELIVERED
    assert link.poll().type == EventType.NONE

    serial.receive(bytes.fromhex("03 01 11 03 22 31 00"))
    event = link.poll()
    assert event.type == EventType.MESSAGE
    assert event.message.type == 1
    assert event.message.delivery == Delivery.UNRELIABLE
    assert event.message.payload == b"\x11\x00\x22"

    serial.transmitted.clear()
    reliable = bytes.fromhex("05 81 07 44 d0 00")
    serial.receive(reliable)
    event = link.poll()
    assert event.type == EventType.MESSAGE
    assert event.message.type == 1
    assert event.message.delivery == Delivery.RELIABLE
    assert event.message.payload == b"\x44"
    assert serial.transmitted == bytes.fromhex("01 03 07 15 00")

    serial.receive(reliable)
    assert link.poll().type == EventType.NONE
    assert serial.transmitted == bytes.fromhex("01 03 07 15 00 01 03 07 15 00")

    retry_serial = FakeSerial()
    retry_link = Link(retry_serial, capacity=8, retry_interval_ms=1)
    assert retry_link.send(1, b"\x33", Delivery.RELIABLE) == SendResult.ACCEPTED
    first_attempt = bytes(retry_serial.transmitted)
    time.sleep(0.002)
    assert retry_link.poll().type == EventType.NONE
    assert retry_serial.transmitted == first_attempt * 2
    retry_link.cancel()
    assert retry_link.send(2, b"", Delivery.UNRELIABLE) == SendResult.ACCEPTED

    recovery_serial = FakeSerial()
    recovery_link = Link(recovery_serial, capacity=8, retry_interval_ms=50)
    recovery_serial.receive(
        bytes.fromhex(
            "03 01 11 03 22 30 00 "
            "05 01 00 "
            "03 01 11 03 22 31 00"
        )
    )
    assert recovery_link.poll().type == EventType.MESSAGE

    recovery_serial.receive(bytes.fromhex("03 01 11"))
    assert recovery_link.poll().type == EventType.NONE
    recovery_serial.receive(bytes.fromhex("03 22 31 00"))
    assert recovery_link.poll().type == EventType.MESSAGE

    crc_serial = FakeSerial()
    crc_link = Link(crc_serial, capacity=8, retry_interval_ms=50)
    assert (
        crc_link.send(ord("1"), b"23456789", Delivery.UNRELIABLE)
        == SendResult.ACCEPTED
    )
    assert crc_serial.transmitted == bytes.fromhex(
        "0b 31 32 33 34 35 36 37 38 39 f4 00"
    )


if __name__ == "__main__":
    test_link_protocol()
