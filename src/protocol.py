"""
protocol.py – Message encoding and decoding for AISR.

Every message is a Python dictionary that MUST contain a 'type' field
(a non-empty string).  Messages are serialised as UTF-8 JSON terminated
by a single newline character (b'\\n').

Functions
---------
encode_message(message: dict) -> bytes
    Validate and serialise a message dictionary.

decode_message(data: bytes) -> dict
    Deserialise and validate a newline-delimited JSON message.
"""

import json
from typing import Any


def encode_message(message: dict) -> bytes:
    """Encode a message dictionary into newline-delimited JSON bytes.

    Parameters
    ----------
    message : dict
        Must contain a ``type`` key whose value is a non-empty string.

    Returns
    -------
    bytes
        UTF-8 encoded JSON followed by exactly one ``\\n`` byte.

    Raises
    ------
    ValueError
        If *message* is not a dict, is missing ``type``, or ``type``
        is not a non-empty string.
    """
    # --- input validation ---------------------------------------------------
    if not isinstance(message, dict):
        raise ValueError(
            f"message must be a dict, got {type(message).__name__}"
        )

    if "type" not in message:
        raise ValueError("message must contain a 'type' field")

    msg_type: Any = message["type"]

    if not isinstance(msg_type, str) or not msg_type.strip():
        raise ValueError(
            "message 'type' must be a non-empty string "
            "(not blank or whitespace-only)"
        )

    # --- serialisation -------------------------------------------------------
    json_str: str = json.dumps(message, ensure_ascii=False)
    return json_str.encode("utf-8") + b"\n"


def decode_message(data: bytes) -> dict:
    """Decode newline-delimited JSON bytes into a message dictionary.

    Parameters
    ----------
    data : bytes
        Raw bytes that must end with exactly one ``\\n`` delimiter.

    Returns
    -------
    dict
        The decoded message dictionary.

    Raises
    ------
    ValueError
        If *data* is not valid UTF-8, does not end with ``\\n``, is not
        valid JSON, or the decoded value is not a dict with a non-empty
        string ``type``.
    """
    # --- raw bytes checks ----------------------------------------------------
    if not isinstance(data, bytes):
        raise ValueError(
            f"data must be bytes, got {type(data).__name__}"
        )

    if not data.endswith(b"\n"):
        raise ValueError(
            "message must end with a newline delimiter (b'\\n')"
        )

    # --- decode UTF-8 --------------------------------------------------------
    try:
        text: str = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"data is not valid UTF-8: {exc}") from exc

    # --- parse JSON ----------------------------------------------------------
    stripped: str = text.rstrip("\n")

    try:
        parsed: Any = json.loads(stripped)
    except json.JSONDecodeError as exc:
        raise ValueError(f"data is not valid JSON: {exc}") from exc

    # --- structure validation ------------------------------------------------
    if not isinstance(parsed, dict):
        raise ValueError(
            f"decoded JSON must be a dict, got {type(parsed).__name__}"
        )

    if "type" not in parsed:
        raise ValueError("decoded message must contain a 'type' field")

    msg_type: Any = parsed["type"]

    if not isinstance(msg_type, str) or not msg_type.strip():
        raise ValueError(
            "decoded message 'type' must be a non-empty string "
            "(not blank or whitespace-only)"
        )

    return parsed
