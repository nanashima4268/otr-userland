# otr-userland

Host software for [OTR-4TS3-U1](https://github.com/nanashima4268/otr-4ts3-u1), an open-hardware 4-tuner ISDB-T/S/C receiver (iCE40 + FX3, USB3).

The software is user-space only (libusb). A daemon (`otrd`) owns the device, and clients (`otr-ts`, `otrctl`) attach over local IPC. It follows the architecture of the Khronos31 `*-userland` family: it builds on [dtv-userland-core](https://github.com/Khronos31/dtv-userland-core) and reuses the CXD2856ER/CXD2858ER and tagged-TS code from [px4-userland](https://github.com/Khronos31/px4-userland), both pinned in `cmake/*.revision`. `otr-ts` follows the common `*-ts` CLI, so BonDriver_Userland and mirakc can use it unchanged.

This is an independent project. It is not affiliated with or endorsed by the px4-userland author.

Status: design stage (see [docs/design.md](docs/design.md)). There is no hardware yet.

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
