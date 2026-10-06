"""FASTLANE card identity: the 10-digit decimal card number.

v2.2.0: the whole firmware identifies a card by its decimal card number, the
number printed on most 125 kHz EM4100 cards and fobs, e.g. ``0008511448``.

RDM6300 frame payload (10 ASCII hex chars), e.g. ``030081DFD8``:
    03        version / customer byte  -> not part of the card number
    0081DFD8  32-bit card value        -> int(.., 16) = 8511448
    card ID   "{:010d}"                -> "0008511448"

Canonical form everywhere (SD database, flash mirror, logs, web API, sync):
a ``str`` of exactly 10 decimal digits, value 0 .. 4294967295. Leading zeros
are part of the ID, so it is never stored or compared as a plain number.

Pure functions, no allocation beyond the returned string; safe to call from
the RFID path.
"""

CARD_ID_DIGITS = 10
CARD_ID_MAX = 0xFFFFFFFF          # 4294967295 -> fits 10 digits and 4 bytes
_DIGITS = "0123456789"


def card_id_from_value(value):
    """32-bit card value (int) -> canonical 10-digit string, or None."""
    try:
        if isinstance(value, bool):
            return None
        value = int(value)
    except Exception:
        return None
    if value < 0 or value > CARD_ID_MAX:
        return None
    return "{:010d}".format(value)


def card_id_value(card_id):
    """Canonical card ID string -> int (no validation beyond int())."""
    return int(card_id)


def normalize_card_id(value):
    """Accept a decimal card number and return the canonical 10-digit form.

    Accepted: "0008511448", "8511448", 8511448 (JSON numbers lose leading
    zeros, so shorter digit strings / ints are zero-padded).
    Rejected (None): hex text, signs, spaces inside, floats, bools, empty,
    more than 10 digits or a value above 4294967295.
    """
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, int):
        return card_id_from_value(value)
    try:
        text = str(value).strip()
    except Exception:
        return None
    if not text or len(text) > CARD_ID_DIGITS:
        return None
    for ch in text:
        if ch not in _DIGITS:
            return None
    return card_id_from_value(int(text))


def card_id_from_frame_hex(data_hex):
    """RDM6300 10-hex payload ("030081DFD8") -> "0008511448", or None.

    The first byte (version) is dropped; the remaining 4 bytes are the card
    number. Only the reader driver and the one-time legacy migration call this.
    """
    try:
        if len(data_hex) != 10:
            return None
        return card_id_from_value(int(data_hex[2:10], 16))
    except Exception:
        return None


def card_id_record(card_id):
    """Canonical card ID -> 4-byte big-endian record (SD / sync format)."""
    value = int(card_id)
    return bytes((
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ))


def card_id_from_record(record):
    """4-byte big-endian record -> canonical card ID, or None."""
    if len(record) != 4:
        return None
    return card_id_from_value(
        (record[0] << 24) | (record[1] << 16) | (record[2] << 8) | record[3]
    )


def card_id_bucket(card_id):
    """Bucket 0..255 = card value mod 256 (low byte; spreads sequential cards)."""
    return int(card_id) & 0xFF
