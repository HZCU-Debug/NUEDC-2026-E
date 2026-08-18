"""验证通信 Demo 的四字节大端计数载荷"""

from receive_counter import decode_counter


def test_decode_counter() -> None:
    assert decode_counter(1, b"\x00\x00\x00\x2a") == 42
    assert decode_counter(2, b"\x00\x00\x00\x2a") is None
    assert decode_counter(1, b"\x00") is None


if __name__ == "__main__":
    test_decode_counter()
