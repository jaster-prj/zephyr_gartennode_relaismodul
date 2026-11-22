import struct
import unittest

from check_firmware import check_image


def image(header=0x400, vector_offset=0x400):
    data = bytearray(0x1000)
    struct.pack_into("<IIHHI", data, 0, 0x96F3B83D, 0, header, 0, 0x800)
    struct.pack_into("<II", data, vector_offset, 0x20004000, 0x08010421)
    return data


class FirmwareLayoutTest(unittest.TestCase):
    def check(self, data):
        check_image(data, 0x400, 0x08010000, 0x30000)

    def test_valid_image(self):
        self.check(image())

    def test_legacy_512_byte_header(self):
        check_image(image(header=0x200, vector_offset=0x200),
                    0x200, 0x08010000, 0x30000)

    def test_old_ci_header_mismatch(self):
        with self.assertRaises(ValueError):
            self.check(image(header=0x200))

    def test_missing_vector_table(self):
        with self.assertRaises(ValueError):
            self.check(image(vector_offset=0x200))

    def test_invalid_reset_vector(self):
        data = image()
        struct.pack_into("<I", data, 0x404, 0x08030421)
        with self.assertRaises(ValueError):
            self.check(data)

    def test_truncated_image(self):
        with self.assertRaises(ValueError):
            self.check(image()[:0x500])

    def test_oversized_image(self):
        with self.assertRaises(ValueError):
            self.check(image() + bytes(0x30000))


if __name__ == "__main__":
    unittest.main()
