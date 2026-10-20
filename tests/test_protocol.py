"""
test_protocol.py – Unit tests for src.protocol.

Run with:
    python -m unittest discover -s tests -v
"""

import json
import unittest

from src.protocol import decode_message, encode_message


class TestEncodeMessage(unittest.TestCase):
    """Tests for encode_message()."""

    # --- happy-path tests ---------------------------------------------------

    def test_encode_valid_command(self):
        """A simple command message encodes to JSON + newline."""
        msg = {"type": "command", "action": "move_forward"}
        result = encode_message(msg)

        # Must end with exactly one newline
        self.assertTrue(result.endswith(b"\n"))

        # Stripping the newline should yield valid JSON
        parsed = json.loads(result.rstrip(b"\n"))
        self.assertEqual(parsed, msg)

    def test_encode_multiple_fields(self):
        """A message with several fields is preserved after encoding."""
        msg = {
            "type": "status",
            "battery": 87,
            "position": [1.5, 2.3, 0.0],
            "active": True,
        }
        result = encode_message(msg)
        parsed = json.loads(result.rstrip(b"\n"))
        self.assertEqual(parsed, msg)

    def test_encode_unicode_content(self):
        """Unicode characters survive encoding without escaping."""
        msg = {"type": "log", "text": "Moteur démarré — vitesse: 42 km/h 🚀"}
        result = encode_message(msg)

        # The raw bytes should contain the actual UTF-8, not \\uXXXX escapes
        self.assertIn("🚀".encode("utf-8"), result)

        parsed = json.loads(result.rstrip(b"\n"))
        self.assertEqual(parsed["text"], msg["text"])

    # --- error-path tests ---------------------------------------------------

    def test_encode_non_dict_raises(self):
        """Passing a non-dict (e.g. a list) must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            encode_message(["not", "a", "dict"])
        self.assertIn("dict", str(ctx.exception).lower())

    def test_encode_missing_type_raises(self):
        """A dict without a 'type' key must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            encode_message({"action": "stop"})
        self.assertIn("type", str(ctx.exception).lower())

    def test_encode_empty_type_raises(self):
        """An empty-string 'type' must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            encode_message({"type": ""})
        self.assertIn("non-empty", str(ctx.exception).lower())

    def test_encode_whitespace_type_raises(self):
        """A whitespace-only 'type' must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            encode_message({"type": "   "})
        self.assertIn("non-empty", str(ctx.exception).lower())

    def test_encode_non_string_type_raises(self):
        """A non-string 'type' (e.g. int) must raise ValueError."""
        with self.assertRaises(ValueError) as ctx:
            encode_message({"type": 123})
        self.assertIn("non-empty", str(ctx.exception).lower())


class TestDecodeMessage(unittest.TestCase):
    """Tests for decode_message()."""

    # --- happy-path tests ---------------------------------------------------

    def test_decode_valid_command(self):
        """A correctly encoded command message decodes back to the dict."""
        original = {"type": "command", "action": "move_forward"}
        encoded = encode_message(original)
        decoded = decode_message(encoded)
        self.assertEqual(decoded, original)

    def test_decode_multiple_fields(self):
        """A multi-field message round-trips correctly."""
        original = {
            "type": "status",
            "battery": 87,
            "position": [1.5, 2.3, 0.0],
            "active": True,
        }
        encoded = encode_message(original)
        decoded = decode_message(encoded)
        self.assertEqual(decoded, original)

    def test_decode_unicode_content(self):
        """Unicode content survives the encode → decode round-trip."""
        original = {"type": "log", "text": "Moteur démarré — vitesse: 42 km/h 🚀"}
        encoded = encode_message(original)
        decoded = decode_message(encoded)
        self.assertEqual(decoded["text"], original["text"])

    # --- error-path tests ---------------------------------------------------

    def test_decode_missing_newline_raises(self):
        """Bytes without a trailing newline must raise ValueError."""
        raw = b'{"type": "command"}'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("newline", str(ctx.exception).lower())

    def test_decode_invalid_json_raises(self):
        """Malformed JSON must raise ValueError."""
        raw = b'{not valid json}\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("json", str(ctx.exception).lower())

    def test_decode_invalid_utf8_raises(self):
        """Bytes that are not valid UTF-8 must raise ValueError."""
        raw = b'\xff\xfe{"type":"x"}\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("utf-8", str(ctx.exception).lower())

    def test_decode_json_list_raises(self):
        """JSON that decodes to a list must raise ValueError."""
        raw = b'[1, 2, 3]\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("dict", str(ctx.exception).lower())

    def test_decode_json_number_raises(self):
        """JSON that decodes to a number must raise ValueError."""
        raw = b'42\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("dict", str(ctx.exception).lower())

    def test_decode_json_string_raises(self):
        """JSON that decodes to a string must raise ValueError."""
        raw = b'"hello"\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("dict", str(ctx.exception).lower())

    def test_decode_json_bool_raises(self):
        """JSON that decodes to a boolean must raise ValueError."""
        raw = b'true\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("dict", str(ctx.exception).lower())

    def test_decode_json_null_raises(self):
        """JSON that decodes to null must raise ValueError."""
        raw = b'null\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("dict", str(ctx.exception).lower())

    def test_decode_missing_type_raises(self):
        """A dict without 'type' must raise ValueError."""
        raw = b'{"action": "stop"}\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("type", str(ctx.exception).lower())

    def test_decode_empty_type_raises(self):
        """An empty-string 'type' must raise ValueError."""
        raw = b'{"type": ""}\n'
        with self.assertRaises(ValueError) as ctx:
            decode_message(raw)
        self.assertIn("non-empty", str(ctx.exception).lower())


if __name__ == "__main__":
    unittest.main()
