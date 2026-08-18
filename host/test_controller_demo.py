from controller_demo import axis_to_velocity, velocity_payload


def test_axis_to_velocity() -> None:
    assert axis_to_velocity(0.05, 0.1) == 0
    assert axis_to_velocity(0.5, 0.1) == 500
    assert axis_to_velocity(-1.0, 0.1) == -1000
    assert axis_to_velocity(0.5, 0.1, invert=True) == -500
    assert axis_to_velocity(2.0, 0.1) == 1000


def test_velocity_payload() -> None:
    assert velocity_payload(1000) == b"\x03\xe8"
    assert velocity_payload(-500) == b"\xfe\x0c"
    assert velocity_payload(0) == b"\x00\x00"

    try:
        velocity_payload(1001)
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-range velocity must fail")


if __name__ == "__main__":
    test_axis_to_velocity()
    test_velocity_payload()
