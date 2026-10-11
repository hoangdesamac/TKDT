import contextlib
import importlib.util
import io
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location(
    "uart_bridge", Path(__file__).resolve().parents[1] / "raspberry" / "uart_bridge.py"
)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class FakeUart:
    def __init__(self, lines=()):
        self.lines = iter(lines)
        self.writes = []

    def readline(self):
        try:
            return next(self.lines)
        except StopIteration:
            raise KeyboardInterrupt

    def write(self, data):
        self.writes.append(data)


class ProtocolTests(unittest.TestCase):
    def test_scan_ids(self):
        self.assertEqual(bridge.parse_scan_request("SCAN,123"), 123)
        self.assertEqual(bridge.parse_scan_request("SCAN,4294967295"), 0xFFFFFFFF)
        for message in (
            "SCAN", "SCAN,0", "SCAN,-1", "SCAN,+1", "SCAN,1,3", "SCAN, 1",
            "SCAN,4294967296", "SCAN,999999999999999999999", "SCAN,\u0661", "PONG",
        ):
            with self.subTest(message=message):
                self.assertIsNone(bridge.parse_scan_request(message))

    def test_all_classes_keep_request_id(self):
        for result in ("1", "2", "3"):
            uart = FakeUart()
            with contextlib.redirect_stdout(io.StringIO()):
                bridge.send_classification(uart, 456, result)
            self.assertEqual(uart.writes, [f"RESULT,456,{result}\n".encode("ascii")])

    def test_invalid_classification(self):
        for scan_id, result in ((0, "3"), (0x100000000, "3"), (1, "4")):
            with self.assertRaises(ValueError):
                bridge.send_classification(FakeUart(), scan_id, result)

    def test_server_responds_only_to_scan(self):
        uart = FakeUart((b"SYSTEM_START\n", b"SCAN,12\n", b"SCAN,0\n", b"SCAN,13\r\n"))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
            bridge.serve_scans(uart, "3")
        self.assertEqual(uart.writes, [b"RESULT,12,3\n", b"RESULT,13,3\n"])

    def test_ping(self):
        uart = FakeUart((b"PONG\r\n",))
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(bridge.check_connection(uart))
        self.assertEqual(uart.writes, [b"P"])

    def test_auto_boot_scan_is_not_lost_during_ping(self):
        uart = FakeUart((b"SCAN,42\n", b"PONG\r\n"))
        pending = []
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(bridge.check_connection(uart, pending))
        self.assertEqual(pending, [42])
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
            bridge.serve_scans(uart, "3", pending)
        self.assertEqual(uart.writes, [b"P", b"RESULT,42,3\n"])


if __name__ == "__main__":
    unittest.main()
