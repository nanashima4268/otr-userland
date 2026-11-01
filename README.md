# otr-userland

Host software for [OTR-4TS3-U1](https://github.com/nanashima4268/otr-4ts3-u1), an open-hardware 4-tuner ISDB-T/S/C receiver (iCE40 + FX3, USB3).

The software is user-space only (libusb). A daemon (`otrd`) owns the device, and clients (`otr-ts`, `otrctl`) attach over local IPC. It follows the architecture of the Khronos31 `*-userland` family: it builds on [dtv-userland-core](https://github.com/Khronos31/dtv-userland-core) and reuses the CXD2856ER/CXD2858ER and tagged-TS code from [px4-userland](https://github.com/Khronos31/px4-userland), both pinned in `cmake/*.revision`. `otr-ts` follows the common `*-ts` CLI, so BonDriver_Userland and mirakc can use it unchanged.

This is an independent project. It is not affiliated with or endorsed by the px4-userland author.

Status: `otrd` builds and passes its offline tests (Linux/macOS). There is no hardware yet, so nothing has run against a real board. See [docs/design.md](docs/design.md).

It does not and will not implement B-CAS/ACAS decryption.

License: GPL-2.0-only (see COPYING), matching the reused px4-userland code.

## Build

```sh
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

The pinned dependencies are fetched automatically. To use local trees instead, pass `-DOTR_DTV_CORE_SOURCE_DIR=...` and `-DOTR_PX4_USERLAND_SOURCE_DIR=...`.

Current contents:
- `src/fx3_protocol.h`: the FX3 vendor-request protocol v1.
- `src/fx3_control.h`: the control-transfer seam.
- `src/fx3_i2c.*`: a px4-userland `BridgeI2cMaster` built on I2C_RUN/I2C_RESULT.
- `tests/fx3_i2c_tests.cpp`: offline tests against a fake FX3. They cover the program encoding, the reused CXD2856ER driver reading and writing through the bridge, channel isolation, NAK handling, and the size limits.
- `tests/ts_path_tests.cpp`: feeds tagged packets (the FPGA mux format), cut into arbitrary USB transfer sizes, to px4-userland's `TaggedTsDemux`. Each receiver must get its packets back bit-exact. A second case starts the stream mid-packet and checks that the demux re-synchronizes. With `OTR_FPGA_SIM_DIR` set, the test demuxes the output of the otr-4ts3-u1 FPGA simulation (`fpga/sim`, `tb_check.py prep-synth`). Verified on 2026-11-01: all 1,599 packets matched.
- `src/fx3_device.*`: implements `Fx3Control` and the dtv-userland-core `Transport` stream contract. Transfers are delivered in submission order; data stays valid until the next `wait_stream`, which resubmits the buffer; DISCONNECTED ends the stream. It sits on a small `UsbBackend` seam.
- `src/libusb_backend.*`: the libusb `UsbBackend`. `--device SERIAL` selects a board by its USB serial (the flash unique ID). With several boards and no serial, it refuses to guess. A dedicated event thread lets the synchronous I2C control transfers and the asynchronous TS stream run side by side. Development VID:PID is 1209:0001, the TS endpoint is EP1 IN (0x81), and stream reads have no timeout because the FX3 sends only full 16 KiB buffers.
- `tests/fx3_device_tests.cpp`: tests the stream contract against a fake backend that has its own event thread. They cover out-of-order completion, buffer lifetime, short transfers, timeout, error resubmit, disconnect, and stop/cancel draining. 50 repeated runs had no failures. The libusb backend was smoke-tested only: init succeeds, and with no device present `open()` returns NOT_FOUND.
- `src/otr_board.*`: board control over the vendor requests: GET_INFO, GPIO, FPGA load and registers, STREAM, the shared CH_ENABLE mask, and the D-50 path reset.
- `src/otr_transport.*`: the `Transport` that px4-userland's `Q3U4StreamDataPlane` drives. It changes the IT930x stream shape to EP 0x81 and 16 KiB multiples. Its start sequence is path reset, then STREAM 1; its stop ends with STREAM 0.
- `src/otr_frontend.*`: the four CXD2856ER/CXD2858ER receivers, modelled on px4-userland's `Mlt5PeFrontend`. Each receiver has its own I2C channel and its own power/reset. The power sequence is power on, 80 ms, release reset, 20 ms. Capture start/stop sets the channel's CH_ENABLE bit. The file also has `OtrTunerBackend` (ISDB-T/S on every receiver, 0 V LNB only) and an absent card reader.
- `tools/otrd.cpp`: the daemon, assembled like px4d's `serve_core`. It has an `EnclosureFamily` and serves endpoints under product `otr-userland`. The instance defaults to the board serial. `--fpga FILE` loads a development bitstream.
- `tests/otrd_tests.cpp`: runs without hardware against a fake board, using the real Fx3Device, OtrBoard, OtrStreamTransport, and `Q3U4StreamDataPlane`. Receiver 2 gets exactly channel 2's packets, in order, with the sync byte restored. The test also checks the D-50 order (path reset before STREAM 1, never while streaming, then STREAM 0), the stream shape, the CH_ENABLE mask, the power sequencing (and power-off after a failed init), and the LNB/frequency limits. 10 repeated runs passed.
