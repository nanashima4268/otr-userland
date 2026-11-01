# otr-userland

Host software for [OTR-4TS3-U1](https://github.com/nanashima4268/otr-4ts3-u1), an open-hardware 4-tuner ISDB-T/S/C receiver (iCE40 + FX3, USB3).

The software is user-space only (libusb). A daemon (`otrd`) owns the device, and clients (`otr-ts`, `otrctl`) attach over local IPC. It follows the architecture of the Khronos31 `*-userland` family: it builds on [dtv-userland-core](https://github.com/Khronos31/dtv-userland-core) and reuses the CXD2856ER/CXD2858ER and tagged-TS code from [px4-userland](https://github.com/Khronos31/px4-userland), both pinned in `cmake/*.revision`. `otr-ts` follows the common `*-ts` CLI, so BonDriver_Userland and mirakc can use it unchanged.

This is an independent project. It is not affiliated with or endorsed by the px4-userland author.

Status: design stage (see [docs/design.md](docs/design.md)). There is no hardware yet.

It does not and will not implement B-CAS/ACAS decryption.

License: GPL-2.0-only (see COPYING), matching the reused px4-userland code.
