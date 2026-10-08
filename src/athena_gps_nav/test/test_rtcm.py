"""Unit tests for athena_gps_nav.rtcm (no hardware, no ROS)."""

from athena_gps_nav.rtcm import RTCM3Framer, crc24q, fragment, msg_type


def make_frame(mtype, payload_len):
    """Build a valid RTCM3 frame: D3, len, [type(12b) + body], CRC24Q."""
    body = bytearray(payload_len)
    body[0] = (mtype >> 4) & 0xFF
    body[1] = (mtype & 0x0F) << 4
    head = bytes([0xD3, (payload_len >> 8) & 0x03, payload_len & 0xFF])
    raw = head + bytes(body)
    return raw + crc24q(raw).to_bytes(3, 'big')


def test_framer_finds_frame_in_junk():
    f = make_frame(1074, 40)
    out = RTCM3Framer().feed(b'\xb5\x62junk$GNGGA' + f + b'\x00\x01')
    assert out == [f]
    assert msg_type(out[0]) == 1074


def test_framer_handles_split_delivery():
    f = make_frame(1005, 19)
    fr = RTCM3Framer()
    assert fr.feed(f[:7]) == []
    assert fr.feed(f[7:]) == [f]


def test_framer_rejects_corrupt_frame():
    f = bytearray(make_frame(1074, 40))
    f[10] ^= 0x01
    fr = RTCM3Framer()
    assert fr.feed(bytes(f)) == []
    assert fr.bad_crc >= 1


def test_fragment_single():
    f = make_frame(1005, 19)
    parts = fragment(f, seq=3)
    assert len(parts) == 1
    flags, n, data = parts[0]
    assert flags == (3 << 3) and n == len(f) and len(data) == 180


def test_fragment_three_chunks():
    f = make_frame(1074, 394)          # 400 bytes on the wire
    assert len(f) == 400
    parts = fragment(f, seq=1)
    assert [p[0] & 0x07 for p in parts] == [1, 3, 5]    # frag bit + id 0,1,2
    assert [p[1] for p in parts] == [180, 180, 40]
    assert all((p[0] >> 3) == 1 for p in parts)


def test_fragment_exact_multiple_gets_terminator():
    f = make_frame(1074, 354)          # 360 bytes = 2 * 180
    parts = fragment(f, seq=0)
    assert [p[1] for p in parts] == [180, 180, 0]


def test_fragment_too_big():
    assert fragment(b'\0' * 721, seq=0) == []
