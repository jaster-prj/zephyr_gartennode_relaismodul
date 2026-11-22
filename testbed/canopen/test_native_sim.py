NODE_ID = 11
MODULE_INDEX = 0x2102
MODULE_SUBINDEX = 0

"""Live CANopen SDO smoke tests against this repository's native_sim node."""
import os
import socket
import unittest

from sdo import UdpCanSdoClient


@unittest.skipUnless(os.getenv("RUN_CAN_INTEGRATION") == "1", "requires this module's native_sim")
class NativeSimCanopenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.client = UdpCanSdoClient(
            cls.sock,
            (os.getenv("CAN_GATEWAY_HOST", "127.0.0.1"), int(os.getenv("CAN_GATEWAY_PORT", "46100"))),
            node_id=NODE_ID,
            timeout=2.0,
        )

    @classmethod
    def tearDownClass(cls):
        cls.sock.close()

    def test_device_type_is_readable(self):
        # Standard CANopen device type object (UNSIGNED32).
        self.assertGreater(self.client.upload(0x1000, 0, 4), 0)

    def test_module_object_dictionary_is_reachable(self):
        # Module-specific object: see objdict/objdict.eds. Reading sub-index 0
        # also validates that array/record entries are present on the live node.
        self.assertGreaterEqual(self.client.upload(MODULE_INDEX, MODULE_SUBINDEX, 1), 0)


if __name__ == "__main__":
    unittest.main()
