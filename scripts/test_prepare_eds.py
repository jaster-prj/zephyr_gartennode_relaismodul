import configparser
from pathlib import Path
import tempfile
import unittest

from prepare_eds import prepare_eds


class ReleaseMetadataTest(unittest.TestCase):
    def test_release_version_matches_image_without_changing_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, source, output = (root / name for name in
                                      ("config", "input.eds", "output.eds"))
            config.write_text('CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="1.3.0+42"\n')
            original = "[100A]\nDefaultValue=0.0.22\nDataType=0x0009\n"
            source.write_text(original)
            prepare_eds(config, source, output)
            eds = configparser.ConfigParser(interpolation=None)
            eds.read(output)
            self.assertEqual(eds.get("100A", "DefaultValue"), "1.3.0+42")
            self.assertEqual(eds.get("100A", "DataType"), "0x0009")
            self.assertEqual(source.read_text(), original)

    def test_missing_version_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "config"
            config.write_text("")
            with self.assertRaises(ValueError):
                prepare_eds(config, config, config)

    def test_missing_software_version_object_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config, source = root / "config", root / "source"
            config.write_text('CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="1.3.0"\n')
            source.write_text("[1000]\nDefaultValue=0\n")
            with self.assertRaises(ValueError):
                prepare_eds(config, source, root / "output")


if __name__ == "__main__":
    unittest.main()
