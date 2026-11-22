from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class UpdateControlTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("cc"), "host C compiler required")
    def test_confirmation_and_upstream_command_guards(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "update_control_test")
            build = subprocess.run(
                ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-DCONFIG_CANOPENNODE_PROGRAM_DOWNLOAD=1",
                 "-DCONFIG_CANOPENNODE_LOG_LEVEL=3",
                 "-DCONFIG_CANOPENNODE_SDO_BUFFER_SIZE=32",
                 '-DCONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="1.3.0"',
                 "-I", str(root / "tests/update_control/stubs"),
                 "-I", str(root / "src"),
                 str(root / "src/update_control.c"),
                 str(root / "tests/update_control/test.c"), "-o", binary],
                capture_output=True, text=True,
            )
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            run = subprocess.run([binary], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
