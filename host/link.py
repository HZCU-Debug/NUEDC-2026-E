import time


def _monotonic_ms():
    return int(time.monotonic() * 1000)


_ticks_ms = getattr(time, "ticks_ms", _monotonic_ms)
_ticks_diff = getattr(time, "ticks_diff", lambda now, then: now - then)


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


class Message:
    __slots__ = ("type", "delivery", "payload")

    def __init__(self, message_type, delivery, payload):
        self.type = message_type
        self.delivery = delivery
        self.payload = payload


_EMPTY_MESSAGE = Message(0, Delivery.UNRELIABLE, b"")


class Event:
    __slots__ = ("type", "message")

    def __init__(self, event_type, message=_EMPTY_MESSAGE):
        self.type = event_type
        self.message = message


_NO_EVENT = Event(EventType.NONE)


def _crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _cobs_encode(data):
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


def _cobs_decode(data):
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
    def __init__(self, serial, capacity, retry_interval_ms):
        if capacity <= 0 or retry_interval_ms <= 0:
            raise ValueError("capacity and retry interval must be positive")
        self._serial = serial
        self._capacity = capacity
        self._retry_interval_ms = retry_interval_ms
        self._frame_capacity = capacity + 3 + (capacity + 3) // 254 + 2
        self._received = bytearray()
        self._discarding = False
        self._pending = None
        self._pending_sequence = 0
        self._last_sent_at = 0
        self._next_sequence = 0
        self._last_received_sequence = None

    def send(self, message_type, payload, delivery):
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
        raw = (
            bytes((wire_type, self._next_sequence)) + payload
            if reliable
            else bytes((wire_type,)) + payload
        )
        raw += bytes((_crc8(raw),))
        frame = _cobs_encode(raw) + b"\0"
        if not self._write(frame):
            return SendResult.WRITE_FAILED
        if reliable:
            self._pending = frame
            self._pending_sequence = self._next_sequence
            self._next_sequence = (self._next_sequence + 1) & 0xFF
            self._last_sent_at = _ticks_ms()
        return SendResult.ACCEPTED

    def poll(self):
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

        now = _ticks_ms()
        if (
            self._pending is not None
            and _ticks_diff(now, self._last_sent_at) >= self._retry_interval_ms
        ):
            self._write(self._pending)
            self._last_sent_at = now
        return _NO_EVENT

    def cancel(self):
        self._pending = None

    def _available(self):
        waiting = getattr(self._serial, "in_waiting", None)
        return waiting if waiting is not None else self._serial.any()

    def _write(self, data):
        return self._serial.write(data) == len(data)

    def _finish_frame(self):
        if self._discarding:
            self._discarding = False
            self._received.clear()
            return _NO_EVENT
        if not self._received:
            return _NO_EVENT

        decoded = _cobs_decode(self._received)
        self._received.clear()
        if decoded is None or len(decoded) < 2 or _crc8(decoded[:-1]) != decoded[-1]:
            return _NO_EVENT

        wire_type = decoded[0]
        if wire_type == 0:
            if (
                len(decoded) == 3
                and self._pending is not None
                and decoded[1] == self._pending_sequence
            ):
                self._pending = None
                return Event(EventType.DELIVERED)
            return _NO_EVENT

        reliable = bool(wire_type & 0x80)
        message_type = wire_type & 0x7F
        offset = 2 if reliable else 1
        if message_type == 0 or len(decoded) < offset + 1:
            return _NO_EVENT
        payload = decoded[offset:-1]
        if len(payload) > self._capacity:
            return _NO_EVENT

        if reliable:
            sequence = decoded[1]
            receipt = bytes((0, sequence))
            self._write(_cobs_encode(receipt + bytes((_crc8(receipt),))) + b"\0")
            if sequence == self._last_received_sequence:
                return _NO_EVENT
            self._last_received_sequence = sequence

        return Event(
            EventType.MESSAGE,
            Message(
                message_type,
                Delivery.RELIABLE if reliable else Delivery.UNRELIABLE,
                bytes(payload),
            ),
        )
