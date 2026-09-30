"""Skydroid TOP protocol helpers used by the C12 driver."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


def add_checksum(body: str) -> str:
    """Append the one-byte ASCII additive checksum as two uppercase hex digits."""
    return f"{body}{sum(body.encode('ascii')) & 0xFF:02X}"


def checksum_ok(packet: str) -> bool:
    packet = packet.strip("\x00\r\n ")
    if len(packet) < 3:
        return False
    try:
        expected = int(packet[-2:], 16)
    except ValueError:
        return False
    return (sum(packet[:-2].encode("ascii")) & 0xFF) == expected


def encode_s16_angle_deg(angle_deg: float) -> str:
    """Encode degrees as signed 16-bit hundredths of a degree."""
    raw = int(round(float(angle_deg) * 100.0))
    return f"{raw & 0xFFFF:04X}"


def decode_s16_angle_deg(hex_text: str) -> float:
    raw = int(hex_text, 16)
    if raw & 0x8000:
        raw -= 0x10000
    return raw / 100.0


def encode_s8_speed_deg_s(speed_deg_s: float) -> str:
    """Encode signed speed in 0.1 degree/s, matching the protocol example."""
    raw = int(round(max(-12.7, min(12.7, float(speed_deg_s))) * 10.0))
    return f"{raw & 0xFF:02X}"


@dataclass(frozen=True)
class GimbalAngles:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float


_GAC_RE = re.compile(r"GAC([0-9A-Fa-f]{12})")


def parse_full_gac(packet: str) -> Optional[GimbalAngles]:
    """Parse a documented full GAC packet.

    Some C12 firmware versions return short GAC packets. Those are deliberately
    left unparsed because their field meaning is not documented.
    """
    match = _GAC_RE.search(packet)
    if match is None:
        return None
    payload = match.group(1)
    return GimbalAngles(
        yaw_deg=decode_s16_angle_deg(payload[0:4]),
        pitch_deg=decode_s16_angle_deg(payload[4:8]),
        roll_deg=decode_s16_angle_deg(payload[8:12]),
    )
