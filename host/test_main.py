from link import Delivery, Event, EventType, SendResult
from main import PuzzleController, ReliablePuzzleSender, build_demo_messages


EXPECTED_MESSAGES = [
    (0x01, bytes.fromhex("04")),
    (0x02, bytes.fromhex("01 00b4 006e 003c 00fa fb2e")),
    (0x02, bytes.fromhex("02 005a 0078 00cd 00ff 11d7")),
    (0x02, bytes.fromhex("03 009b 0082 0050 00c8 dcd8")),
    (0x02, bytes.fromhex("04 0064 008c 00be 00c3 4650")),
]


class FakeLink:
    def __init__(self) -> None:
        self.sent: list[tuple[int, bytes, int]] = []
        self.next_event = Event(EventType.NONE)

    def send(self, message_type: int, payload: bytes, delivery: int) -> int:
        self.sent.append((message_type, payload, delivery))
        return SendResult.ACCEPTED

    def poll(self) -> Event:
        event = self.next_event
        self.next_event = Event(EventType.NONE)
        return event

    def deliver(self) -> None:
        self.next_event = Event(EventType.DELIVERED)

    def request_mode(self, mode: int) -> None:
        from link import Message

        self.next_event = Event(
            EventType.MESSAGE,
            Message(0x03, Delivery.RELIABLE, bytes((mode,))),
        )


def test_host_uses_puzzle_protocol() -> None:
    assert build_demo_messages() == EXPECTED_MESSAGES


def test_sender_waits_for_each_delivery() -> None:
    link = FakeLink()
    sender = ReliablePuzzleSender(link, build_demo_messages())

    for expected_count in range(1, len(EXPECTED_MESSAGES) + 1):
        sender.step()
        assert link.sent == [
            (*message, Delivery.RELIABLE)
            for message in EXPECTED_MESSAGES[:expected_count]
        ]
        sender.step()
        assert len(link.sent) == expected_count
        link.deliver()
        sender.step()

    assert sender.complete
    assert sender.delivered_count == len(EXPECTED_MESSAGES)


def test_controller_waits_for_mode_request() -> None:
    link = FakeLink()
    controller = PuzzleController(link)

    controller.step()
    assert link.sent == []

    link.request_mode(1)
    assert controller.step() == 1
    assert link.sent == [(*EXPECTED_MESSAGES[0], Delivery.RELIABLE)]


def test_controller_toggles_vision_calibration() -> None:
    link = FakeLink()
    controller = PuzzleController(link)

    link.request_mode(0)
    assert controller.step() == 0
    assert controller.calibration_active
    assert link.sent == []

    link.request_mode(0)
    assert controller.step() == 0
    assert not controller.calibration_active
    assert link.sent == []

    link.request_mode(1)
    assert controller.step() == 1
    assert link.sent == [(*EXPECTED_MESSAGES[0], Delivery.RELIABLE)]


if __name__ == "__main__":
    test_host_uses_puzzle_protocol()
    test_sender_waits_for_each_delivery()
    test_controller_waits_for_mode_request()
    test_controller_toggles_vision_calibration()
