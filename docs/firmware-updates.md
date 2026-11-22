# Relay module firmware updates

The update fixes are ported from the cistern module. Cistern hardware tests
do not establish that the relay module has passed a hardware update test.
Node ID remains **11** (`0x0b`); relay mapping, output polarity and NMT startup
policy are unchanged. Existing local native-sim source changes and the modified
`custom_boards` submodule are preserved.

## Build and release

Use the Gartennode west workspace and matching private signing key at
`bootloader/mcuboot/gartennode.pem`. Never commit or publish the key.
From the workspace root:

```sh
west build -b gartennode application -d build/relay
python3 application/scripts/check_firmware.py \
  build/relay/zephyr/zephyr.signed.bin build/relay/zephyr/.config
python3 bootloader/mcuboot/scripts/imgtool.py verify \
  --key bootloader/mcuboot/gartennode.pem build/relay/zephyr/zephyr.signed.bin
python3 application/scripts/prepare_eds.py \
  build/relay/zephyr/.config application/objdict/objdict.eds objdict.eds
python3 -m unittest discover -s application/scripts -p 'test_*.py'
```

CI checks out this repository as `app` rather than `application`; its commands
use that path. Publish Zephyr's `zephyr.signed.bin`, not an image re-signed with
a hard-coded header size. Zephyr 4.5 uses a `0x400` STM32 header.
The default signed version is `1.3.1`; override
`CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` for each new release.
The running `0x100A` string and generated release EDS use that same version.

The application uses current `CANOPENNODE` Kconfig names; the obsolete
`CAN_MAX_FILTER` assignment was removed for Zephyr 4.5 compatibility.

Both application and supplied MCUboot configuration use **swap using scratch**:

| Region | Address/offset | Size |
| --- | --- | --- |
| MCUboot, internal | `0x08000000` | 64 KiB |
| Primary, internal | `0x08010000` | 192 KiB |
| Secondary, external | `0` | 192 KiB |
| Scratch, external | `0x30000` | 192 KiB |

Build matching MCUboot separately:

```sh
west build -b gartennode bootloader/mcuboot/boot/zephyr \
  -d build/relay-boot \
  -- -DEXTRA_CONF_FILE="$PWD/application/bootloader.conf"
```

Verify the installed bootloader's layout and signing key before deployment.
The supplied bootloader disables watchdog support. This port does not add a
watchdog to the application or change relay initialization.

## CAN update

Use node `0xb` and the uploaded image's version:

```sh
curl --cacert CA.pem \
  -H 'Content-Type: application/octet-stream' \
  --data-binary @build/relay/zephyr/zephyr.signed.bin \
  'https://YOUR_GATEWAY/canopenrest/api/v1/flash?node=0xb&version=1.3.1'
```

Poll `GET /canopenrest/api/v1/flash?id=<returned-id>` to completion.
Require reboot, matching `0x100A`, and explicit confirmation—not just successful
data reception.

STOP/CLEAR/START retain upstream PRE-OPERATIONAL restrictions. CLEAR erases the
secondary slot; reception-time stream-flash erasing is disabled to avoid
erasing it again during SDO handling.

The adapter permits explicit `0x1F51:1 = 0x80` confirmation while OPERATIONAL
as well as PRE-OPERATIONAL, without requiring another NMT transition after
gateway verification. It never automatically confirms an image; MCUboot
rollback remains available for unconfirmed test images.

The cistern's swap/startup took about 16.8 seconds, longer than the gateway's
fixed ten-second initial wait. Allow adequate reboot/verification time;
relay module timing has not been measured.

## SWD recovery

Back up internal flash before recovery. Use a **full padded BIN**, not sparse
HEX: sparse programming can leave old swap-status bytes in a primary trailer
page and cause MCUboot to interpret an invalid interrupted swap.

Generate a confirmed baseline separately:

```sh
west build -b gartennode application -d build/relay-recovery \
  -- -DCONFIG_MCUBOOT_GENERATE_CONFIRMED_IMAGE=y
```

Verify layout/signature and exact `0x30000`-byte length before programming
`zephyr.signed.confirmed.bin` at `0x08010000`. Resolve pending external swap
state first; do not resume a pending swap against an unrelated replacement
primary. Avoid mass erases of the bootloader or external settings.
Do not use another module's firmware as a relay module recovery image.

## Optional diagnostics

Add `-DEXTRA_CONF_FILE=debug/update.conf` when building the application.
It records reset causes, download byte counts and callback latency using
deferred RTT logs, filtering GPIO-expander log noise without disabling relays.
Read `relay_update_diagnostics` and RTT only after a transfer;
live SWD reads or halts can disturb timing.

`debug/bootloader.conf` is an optional second MCUboot fragment for RTT output.

## Port validation and deployment caveat

Normal firmware, diagnostic firmware and matching MCUboot build with Zephyr
4.5.0. Both application images are 196608 bytes and pass layout/vector-table
and signature verification; generated EDS version is `1.3.1`.
All 11 host update regression tests pass. No hardware was flashed.

**Existing relay bug:** the hardware `writeOutput()` loop iterates eight times,
but `outputLookupTable` has six entries. The compiler warns of undefined
behavior at iteration six. This port preserves that code; correct and test the
relay bounds before deploying these images. A successful build/signature check
does not establish safe relay operation.

Native-sim source changes were preserved, but this checkout has no native-sim
board configuration or overlay, so a complete native-sim build/runtime test
was not performed as part of this port. The added update sources compile out
when program download is disabled.
