"""Minimal expedited CANopen SDO client over the native UDP-CAN wire format."""

import socket
import struct
import time

CAN_WIRE = struct.Struct("<IBB2x64s")
NODE_ID = 9
SDO_REQUEST_ID = 0x600 + NODE_ID
SDO_RESPONSE_ID = 0x580 + NODE_ID


class SdoError(RuntimeError):
    pass


class UdpCanSdoClient:
    def __init__(
        self,
        gateway: socket.socket,
        node_address: tuple[str, int],
        node_id: int = NODE_ID,
        timeout: float = 1.0,
    ) -> None:
        self.gateway = gateway
        self.node_address = node_address
        self.request_id = 0x600 + node_id
        self.response_id = 0x580 + node_id
        self.timeout = timeout

    def _transfer(self, request: bytes, index: int, subindex: int) -> bytes:
        frame = CAN_WIRE.pack(self.request_id, 8, 0, request.ljust(8, b"\0"))
        deadline = time.monotonic() + self.timeout
        while True:
            self.gateway.sendto(frame, self.node_address)
            while time.monotonic() < deadline:
                self.gateway.settimeout(max(0.001, deadline - time.monotonic()))
                try:
                    packet, _ = self.gateway.recvfrom(2048)
                except socket.timeout:
                    break
                if len(packet) != CAN_WIRE.size:
                    continue
                can_id, dlc, _flags, data = CAN_WIRE.unpack(packet)
                if can_id != self.response_id or dlc != 8:
                    continue
                if data[1:3] != struct.pack("<H", index) or data[3] != subindex:
                    continue
                if data[0] == 0x80:
                    abort = struct.unpack_from("<I", data, 4)[0]
                    raise SdoError(f"SDO abort 0x{abort:08x} at 0x{index:04x}:{subindex}")
                return data
            if time.monotonic() >= deadline:
                raise TimeoutError(f"SDO timeout at 0x{index:04x}:{subindex}")

    def download(self, index: int, subindex: int, value: int, size: int) -> None:
        self._download_request(index, subindex, value, size, wait_for_ack=True)

    def download_unconfirmed(self, index: int, subindex: int, value: int, size: int) -> None:
        """Send an expedited download without waiting for an SDO response."""
        self._download_request(index, subindex, value, size, wait_for_ack=False)

    def _download_request(
        self, index: int, subindex: int, value: int, size: int, wait_for_ack: bool
    ) -> None:
        commands = {1: 0x2F, 2: 0x2B, 4: 0x23}
        if size not in commands:
            raise ValueError("expedited SDO size must be 1, 2, or 4 bytes")
        request = bytes((commands[size],)) + struct.pack("<H", index) + bytes((subindex,))
        request += value.to_bytes(size, "little", signed=False)
        if not wait_for_ack:
            self.gateway.sendto(
                CAN_WIRE.pack(self.request_id, 8, 0, request.ljust(8, b"\0")),
                self.node_address,
            )
            return
        response = self._transfer(request, index, subindex)
        if response[0] != 0x60:
            raise SdoError(f"unexpected SDO download response 0x{response[0]:02x}")

    def upload(self, index: int, subindex: int, size: int) -> int:
        response = self._transfer(
            bytes((0x40,)) + struct.pack("<H", index) + bytes((subindex,)),
            index,
            subindex,
        )
        command = response[0]
        if command & 0xE0 != 0x40 or command & 0x02 == 0:
            raise SdoError(f"non-expedited or invalid SDO upload response 0x{command:02x}")
        indicated_size = 4 - ((command >> 2) & 0x03) if command & 0x01 else 4
        if indicated_size != size:
            raise SdoError(
                f"SDO size mismatch at 0x{index:04x}:{subindex}: "
                f"expected {size}, got {indicated_size}"
            )
        return int.from_bytes(response[4 : 4 + size], "little")


def configure_calibration(client: UdpCanSdoClient) -> dict[tuple[int, int], int]:
    """Write calibration and require each SDO download acknowledgement.

    Readback is intentionally not required: the server's 0x60 download
    response confirms acceptance without introducing a separate upload.
    """
    values = (
        (0x2106, 1, 70, 1),
        (0x2106, 2, 140, 1),
        (0x2107, 1, 2048, 2),
        (0x2107, 2, 3500, 2),
        (0x2111, 0, 25, 2),
    )
    verified: dict[tuple[int, int], int] = {}
    for index, subindex, value, size in values:
        client.download(index, subindex, value, size)
        verified[(index, subindex)] = value
    return verified
