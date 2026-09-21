"""PC probe unit tests; no pyserial, serial port, or hardware required."""
import importlib.util
from pathlib import Path
import struct
import unittest
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("imu_probe", ROOT / "Tools/imu_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class ProbeTests(unittest.TestCase):
    def test_official_ping(self):
        self.assertEqual(probe.packet(1, 1), bytes.fromhex("ff ff fd 00 01 03 00 01 19 4e"))

    def test_stuffing(self):
        frame = probe.packet(200, 3, bytes.fromhex("ff ff fd 00 ff ff fd"))
        self.assertEqual(frame[7:-2], bytes.fromhex("03 ff ff fd fd 00 ff ff fd fd"))
        self.assertEqual(len(frame), 7 + int.from_bytes(frame[5:7], "little"))

    def test_diagnostic_decode(self):
        data = bytearray(116)
        struct.pack_into("<3h3e", data, 0, -32768, 0, 32767, .5, -.5, 0)
        data[19:24] = bytes([3, 0x70, 0, 0, 1])
        struct.pack_into("<22I", data, 24, *range(22))
        decoded = probe.decode(data)
        self.assertTrue(decoded["ready"])
        self.assertAlmostEqual(decoded["gyro_x_dps"], -573.44)
        self.assertAlmostEqual(decoded["q_w"], 2 ** -.5)
        self.assertEqual(decoded["clock_hz"], 19)
        self.assertEqual(decoded["log_dropped"], 21)

    def test_invalid_quaternion(self):
        data = bytearray(116)
        data[19] = 1
        struct.pack_into("<e", data, 6, float("nan"))
        self.assertFalse(probe.decode(data)["ready"])


if __name__ == "__main__":
    unittest.main()
