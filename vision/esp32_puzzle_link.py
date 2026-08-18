"""ESP32 puzzle protocol for the Raspberry Pi vision controller.

Wire format and reliable-delivery behavior match NUEDC-2026/host/link.py.
"""

from __future__ import annotations

import struct
import time
from math import ceil, floor
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


MODE_REQUEST_MESSAGE = 0x03
CANCEL_REQUEST_VALUE = 0xFF
PIECE_COUNT_MESSAGE = 0x01
PIECE_DATA_MESSAGE = 0x02
LINK_CAPACITY = 16
PIECE_FORMAT = ">Bhhhhh"
PIECE_PAYLOAD_SIZE = struct.calcsize(PIECE_FORMAT)


class Delivery:
    UNRELIABLE = 0
    RELIABLE = 1


class SendResult:
    ACCEPTED = 0
    BUSY = 1
    INVALID_ARGUMENT = 2
    PAYLOAD_TOO_LARGE = 3
    WRITE_FAILED = 4


class EventType:
    NONE = 0
    MESSAGE = 1
    DELIVERED = 2


@dataclass(frozen=True)
class Message:
    type: int
    delivery: int
    payload: bytes


@dataclass(frozen=True)
class Event:
    type: int
    message: Optional[Message] = None


NO_EVENT = Event(EventType.NONE)


def crc8(data: Iterable[int]) -> int:
    """CRC-8, polynomial 0x07, init/xorout 0x00, MSB first."""
    crc = 0
    for byte in data:
        crc ^= int(byte)
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray((0,))
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> Optional[bytes]:
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        end = index + code - 1
        if code == 0 or end > len(data):
            return None
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


class Link:
    """Bidirectional COBS/CRC link with one reliable packet in flight."""

    def __init__(
        self,
        serial_connection,
        capacity: int = LINK_CAPACITY,
        retry_interval_ms: int = 50,
    ) -> None:
        if capacity <= 0 or retry_interval_ms <= 0:
            raise ValueError("capacity and retry interval must be positive")
        self._serial = serial_connection
        self._capacity = capacity
        self._retry_interval_ms = retry_interval_ms
        self._frame_capacity = capacity + 3 + (capacity + 3) // 254 + 2
        self._received = bytearray()
        self._discarding = False
        self._pending: Optional[bytes] = None
        self._pending_sequence = 0
        self._last_sent_at = 0
        self._next_sequence = 0
        self._last_received_sequence: Optional[int] = None
        self._retransmission_count = 0

    @property
    def waiting_for_ack(self) -> bool:
        return self._pending is not None

    @property
    def retransmission_count(self) -> int:
        return self._retransmission_count

    def send(self, message_type: int, payload, delivery: int) -> int:
        if self._pending is not None:
            return SendResult.BUSY
        if (
            not isinstance(message_type, int)
            or not 1 <= message_type <= 0x7F
            or delivery not in (Delivery.UNRELIABLE, Delivery.RELIABLE)
        ):
            return SendResult.INVALID_ARGUMENT
        if isinstance(payload, int):
            return SendResult.INVALID_ARGUMENT
        try:
            payload = bytes(payload)
        except (TypeError, ValueError):
            return SendResult.INVALID_ARGUMENT
        if len(payload) > self._capacity:
            return SendResult.PAYLOAD_TOO_LARGE

        reliable = delivery == Delivery.RELIABLE
        wire_type = message_type | (0x80 if reliable else 0)
        if reliable:
            raw = bytes((wire_type, self._next_sequence)) + payload
        else:
            raw = bytes((wire_type,)) + payload
        raw += bytes((crc8(raw),))
        frame = cobs_encode(raw) + b"\0"
        if not self._write(frame):
            return SendResult.WRITE_FAILED
        if reliable:
            self._pending = frame
            self._pending_sequence = self._next_sequence
            self._next_sequence = (self._next_sequence + 1) & 0xFF
            self._last_sent_at = self._monotonic_ms()
        return SendResult.ACCEPTED

    def poll(self) -> Event:
        while self._available():
            data = self._serial.read(1)
            if not data:
                break
            byte = data if isinstance(data, int) else data[0]
            if byte == 0:
                event = self._finish_frame()
                if event.type != EventType.NONE:
                    return event
            elif not self._discarding:
                if len(self._received) < self._frame_capacity:
                    self._received.append(byte)
                else:
                    self._received.clear()
                    self._discarding = True

        now = self._monotonic_ms()
        if (
            self._pending is not None
            and now - self._last_sent_at >= self._retry_interval_ms
        ):
            self._write(self._pending)
            self._last_sent_at = now
            self._retransmission_count += 1
        return NO_EVENT

    def cancel(self) -> None:
        self._pending = None

    def discard_received(self) -> None:
        """Discard stale complete or partial frames after a finished task."""
        self._received.clear()
        self._discarding = False
        reset_input = getattr(
            self._serial,
            "reset_input_buffer",
            None,
        )
        if reset_input is not None:
            reset_input()
            return
        while self._available():
            if not self._serial.read(1):
                break

    @staticmethod
    def _monotonic_ms() -> int:
        return int(time.monotonic() * 1000)

    def _available(self) -> int:
        waiting = getattr(self._serial, "in_waiting", None)
        if waiting is not None:
            return int(waiting)
        return int(self._serial.any())

    def _write(self, data: bytes) -> bool:
        return self._serial.write(data) == len(data)

    def _finish_frame(self) -> Event:
        if self._discarding:
            self._discarding = False
            self._received.clear()
            return NO_EVENT
        if not self._received:
            return NO_EVENT

        decoded = cobs_decode(bytes(self._received))
        self._received.clear()
        if (
            decoded is None
            or len(decoded) < 2
            or crc8(decoded[:-1]) != decoded[-1]
        ):
            return NO_EVENT

        wire_type = decoded[0]
        if wire_type == 0:
            if (
                len(decoded) == 3
                and self._pending is not None
                and decoded[1] == self._pending_sequence
            ):
                self._pending = None
                return Event(EventType.DELIVERED)
            return NO_EVENT

        reliable = bool(wire_type & 0x80)
        message_type = wire_type & 0x7F
        offset = 2 if reliable else 1
        if message_type == 0 or len(decoded) < offset + 1:
            return NO_EVENT
        payload = decoded[offset:-1]
        if len(payload) > self._capacity:
            return NO_EVENT

        if reliable:
            sequence = decoded[1]
            receipt = bytes((0, sequence))
            receipt += bytes((crc8(receipt),))
            self._write(cobs_encode(receipt) + b"\0")
            if sequence == self._last_received_sequence:
                return NO_EVENT
            self._last_received_sequence = sequence

        return Event(
            EventType.MESSAGE,
            Message(
                message_type,
                Delivery.RELIABLE if reliable else Delivery.UNRELIABLE,
                bytes(payload),
            ),
        )


def parse_mode_request(event: Event) -> Optional[int]:
    """Return mode 0..4 for a valid reliable request, otherwise None."""
    if (
        event.type != EventType.MESSAGE
        or event.message is None
        or event.message.type != MODE_REQUEST_MESSAGE
        or event.message.delivery != Delivery.RELIABLE
        or len(event.message.payload) != 1
    ):
        return None
    mode = event.message.payload[0]
    return mode if mode in (0, 1, 2, 3, 4) else None


def is_cancel_request(event: Event) -> bool:
    """Return True for reliable TYPE=0x03, PAYLOAD=0xFF cancellation."""
    return bool(
        event.type == EventType.MESSAGE
        and event.message is not None
        and event.message.type == MODE_REQUEST_MESSAGE
        and event.message.delivery == Delivery.RELIABLE
        and event.message.payload == bytes((CANCEL_REQUEST_VALUE,))
    )


def _rounded_int16(value: float, field: str) -> int:
    numeric = float(value)
    result = (
        int(floor(numeric + 0.5))
        if numeric >= 0.0
        else int(ceil(numeric - 0.5))
    )
    if not -32768 <= result <= 32767:
        raise ValueError("{} outside int16 range: {}".format(field, value))
    return result


def build_result_messages(actions: Sequence[dict]) -> list[tuple[int, bytes]]:
    """Convert a state-machine plan to ESP32 count and piece messages."""
    if not 1 <= len(actions) <= 4:
        raise ValueError("ESP32 accepts 1..4 pieces, got {}".format(len(actions)))

    messages: list[tuple[int, bytes]] = [
        (PIECE_COUNT_MESSAGE, bytes((len(actions),)))
    ]
    for transmitted_id, action in enumerate(actions, start=1):
        source = action["source_pickup_mm"]
        # T is the end position of the same physical point held at S.
        # Never substitute the target centroid: doing so introduces a
        # translation error whenever the safe pickup point is off-centre.
        if "target_pickup_mm" not in action:
            raise ValueError(
                "piece {} is missing target_pickup_mm".format(
                    transmitted_id
                )
            )
        target = action["target_pickup_mm"]
        source_x = _rounded_int16(source[0], "source_x")
        source_y = _rounded_int16(source[1], "source_y")
        target_x = _rounded_int16(target[0], "target_x")
        target_y = _rounded_int16(target[1], "target_y")
        angle_centideg = _rounded_int16(
            float(action["rotation_delta_deg"]) * 100.0,
            "rotation_centideg",
        )
        if not (
            0 <= source_x <= 210
            and 0 <= target_x <= 210
            and 0 <= source_y <= 297
            and 0 <= target_y <= 297
        ):
            raise ValueError(
                "piece {} coordinate outside A4: source=({},{}), "
                "target=({},{})".format(
                    transmitted_id,
                    source_x,
                    source_y,
                    target_x,
                    target_y,
                )
            )
        payload = struct.pack(
            PIECE_FORMAT,
            transmitted_id,
            source_x,
            source_y,
            target_x,
            target_y,
            angle_centideg,
        )
        messages.append((PIECE_DATA_MESSAGE, payload))
    return messages


class ReliablePuzzleSender:
    """Send one result packet at a time and wait for each ESP32 ACK."""

    def __init__(self, link: Link) -> None:
        self._link = link
        self._messages: tuple[tuple[int, bytes], ...] = ()
        self._index = 0
        self._waiting = False
        self.delivered_count = 0

    @property
    def active(self) -> bool:
        return bool(self._messages)

    @property
    def total_count(self) -> int:
        return len(self._messages)

    @property
    def complete(self) -> bool:
        return (
            bool(self._messages)
            and self._index >= len(self._messages)
            and not self._waiting
        )

    def start(self, actions: Sequence[dict]) -> None:
        if self.active and not self.complete:
            raise RuntimeError("previous puzzle result is still being sent")
        self._messages = tuple(build_result_messages(actions))
        self._index = 0
        self._waiting = False
        self.delivered_count = 0

    def step(self, event: Event = NO_EVENT) -> None:
        if event.type == EventType.DELIVERED and self._waiting:
            self._waiting = False
            self._index += 1
            self.delivered_count += 1
        if (
            not self._messages
            or self._waiting
            or self._index >= len(self._messages)
        ):
            return
        message_type, payload = self._messages[self._index]
        result = self._link.send(
            message_type,
            payload,
            Delivery.RELIABLE,
        )
        if result == SendResult.ACCEPTED:
            self._waiting = True
        elif result != SendResult.BUSY:
            raise RuntimeError(
                "failed to send result packet: {}".format(result)
            )

    def clear(self) -> None:
        self._link.cancel()
        self._messages = ()
        self._index = 0
        self._waiting = False
        self.delivered_count = 0
