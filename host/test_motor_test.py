from motor_test import position_payload, speed_payload


def test_speed_payload() -> None:
    assert speed_payload("X", 300) == b"X\x01\x2c"
    assert speed_payload("r", -300) == b"R\xfe\xd4"
    assert speed_payload("Z", 0) == b"Z\x00\x00"

    for axis, rpm in (("A", 0), ("X", -6001), ("X", 6001)):
        try:
            speed_payload(axis, rpm)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid motor speed command must fail")


def test_position_payload() -> None:
    assert position_payload("X", 90.0, 300) == b"XB\xb4\x00\x00\x01\x2c"
    assert position_payload("r", -360.0, 6000) == b"R\xc3\xb4\x00\x00\x17p"

    for axis, degrees, rpm in (
        ("A", 0.0, 300),
        ("X", float("inf"), 300),
        ("X", 90.0, 0),
        ("X", 90.0, 6001),
    ):
        try:
            position_payload(axis, degrees, rpm)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid motor position command must fail")


if __name__ == "__main__":
    test_speed_payload()
    test_position_payload()
