"""
test_protocol.py – Unit tests for src.protocol and TCP integration.

Run with:
    python -m unittest discover -s tests -v
"""

import json
import socket
import threading
import time
import unittest

from src.protocol import MessageBuffer, decode_message, encode_message
from src.tcp_server import handle_message


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


# ============================================================================
# NEW: MessageBuffer tests
# ============================================================================


class TestMessageBuffer(unittest.TestCase):
    """Tests for the MessageBuffer TCP framing helper."""

    def test_single_complete_message(self):
        """A single complete message is returned immediately."""
        buf = MessageBuffer()
        msg = {"type": "PING"}
        buf.feed(encode_message(msg))
        result = buf.get_messages()
        self.assertEqual(result, [msg])

    def test_fragmented_message(self):
        """A message split across two feed() calls is reassembled."""
        buf = MessageBuffer()
        raw = encode_message({"type": "PING"})
        mid = len(raw) // 2

        buf.feed(raw[:mid])
        self.assertEqual(buf.get_messages(), [])  # not yet complete

        buf.feed(raw[mid:])
        result = buf.get_messages()
        self.assertEqual(result, [{"type": "PING"}])

    def test_multiple_messages_in_one_feed(self):
        """Two messages arriving in a single recv() are both extracted."""
        buf = MessageBuffer()
        m1 = encode_message({"type": "PING"})
        m2 = encode_message({"type": "STATUS", "battery": 42})

        buf.feed(m1 + m2)
        result = buf.get_messages()
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], {"type": "PING"})
        self.assertEqual(result[1], {"type": "STATUS", "battery": 42})

    def test_partial_message_kept_in_buffer(self):
        """Trailing partial bytes survive across get_messages() calls."""
        buf = MessageBuffer()
        full = encode_message({"type": "A"})
        partial = b'{"type": "B"'  # no newline

        buf.feed(full + partial)
        result1 = buf.get_messages()
        self.assertEqual(len(result1), 1)
        self.assertEqual(result1[0], {"type": "A"})

        # Now complete the partial message
        buf.feed(b'}\n')
        result2 = buf.get_messages()
        self.assertEqual(len(result2), 1)
        self.assertEqual(result2[0], {"type": "B"})

    def test_invalid_message_skipped(self):
        """An invalid message is silently skipped; valid ones still work."""
        buf = MessageBuffer()
        bad = b'not json at all\n'
        good = encode_message({"type": "OK"})

        buf.feed(bad + good)
        result = buf.get_messages()
        self.assertEqual(result, [{"type": "OK"}])

    def test_empty_feed(self):
        """Feeding empty bytes produces no messages and doesn't crash."""
        buf = MessageBuffer()
        buf.feed(b"")
        self.assertEqual(buf.get_messages(), [])

    def test_three_fragments(self):
        """A message split into three fragments is reassembled."""
        buf = MessageBuffer()
        raw = encode_message({"type": "CMD", "speed": 10})
        third = len(raw) // 3

        buf.feed(raw[:third])
        self.assertEqual(buf.get_messages(), [])

        buf.feed(raw[third:2 * third])
        self.assertEqual(buf.get_messages(), [])

        buf.feed(raw[2 * third:])
        result = buf.get_messages()
        self.assertEqual(result, [{"type": "CMD", "speed": 10}])


# ============================================================================
# NEW: handle_message tests
# ============================================================================


class TestHandleMessage(unittest.TestCase):
    """Tests for the server's handle_message() function."""

    def test_ack_contains_received_type(self):
        """The ACK response includes the type of the original message."""
        response = handle_message({"type": "PING"})
        self.assertEqual(response["type"], "ACK")
        self.assertEqual(response["received_type"], "PING")
        self.assertEqual(response["status"], "ok")

    def test_ack_for_arbitrary_type(self):
        """Any valid message type produces an ACK."""
        response = handle_message({"type": "STATUS", "extra": 123})
        self.assertEqual(response["type"], "ACK")
        self.assertEqual(response["received_type"], "STATUS")


# ============================================================================
# NEW: TCP integration test
# ============================================================================


class TestTCPIntegration(unittest.TestCase):
    """End-to-end test: real TCP connection over loopback."""

    def _run_echo_server(
        self, server_sock: socket.socket, ready: threading.Event
    ) -> None:
        """Mini server loop for integration tests."""
        ready.set()
        conn, _ = server_sock.accept()
        conn.settimeout(5)
        buf = MessageBuffer()
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf.feed(data)
                for msg in buf.get_messages():
                    resp = handle_message(msg)
                    conn.sendall(encode_message(resp))
        except socket.timeout:
            pass
        finally:
            conn.close()

    def test_ping_pong_over_tcp(self):
        """Send a PING and receive a structured ACK over a real socket."""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", 0))  # OS-assigned port
        server_sock.listen(1)
        port = server_sock.getsockname()[1]

        ready = threading.Event()
        t = threading.Thread(
            target=self._run_echo_server,
            args=(server_sock, ready),
            daemon=True,
        )
        t.start()
        ready.wait(timeout=5)

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            client.connect(("127.0.0.1", port))

            # Send a PING
            client.sendall(encode_message({"type": "PING"}))
            time.sleep(0.1)

            # Read response
            buf = MessageBuffer()
            data = client.recv(4096)
            buf.feed(data)
            responses = buf.get_messages()

            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["type"], "ACK")
            self.assertEqual(responses[0]["received_type"], "PING")

            client.close()
        finally:
            server_sock.close()
            t.join(timeout=5)

    def test_multiple_messages_over_tcp(self):
        """Send two messages rapidly and receive two ACKs."""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", 0))
        server_sock.listen(1)
        port = server_sock.getsockname()[1]

        ready = threading.Event()
        t = threading.Thread(
            target=self._run_echo_server,
            args=(server_sock, ready),
            daemon=True,
        )
        t.start()
        ready.wait(timeout=5)

        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5)
            client.connect(("127.0.0.1", port))

            # Send two messages in one sendall
            m1 = encode_message({"type": "PING"})
            m2 = encode_message({"type": "STATUS"})
            client.sendall(m1 + m2)
            time.sleep(0.2)

            # Collect all response data
            buf = MessageBuffer()
            data = client.recv(4096)
            buf.feed(data)
            responses = buf.get_messages()

            self.assertEqual(len(responses), 2)
            self.assertEqual(responses[0]["received_type"], "PING")
            self.assertEqual(responses[1]["received_type"], "STATUS")

            client.close()
        finally:
            server_sock.close()
            t.join(timeout=5)


if __name__ == "__main__":
    unittest.main()
