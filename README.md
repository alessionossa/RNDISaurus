# RNDISaurus — USB tethering for macOS

*RNDISaurus* 🦖 brings RNDIS back from extinction on macOS allowing you to use your Android device's USB tethering functionality on macOS.

Modern Android phones, starting with Google Pixel devices, support CDC-NCM to allow native plug-and-play tethering on Linux and macOS without proprietary drivers, but many Android devices around are not compatible with this protocol.

RNDISaurus installs a DriverKit driver extension that gives the phone a native Ethernet
interface, with no kernel extension and no reduced security.

## Status

**Pre-release.** Apple has granted the DriverKit entitlements, and a
[public beta](https://testflight.apple.com/join/xFjUzdEw) is on TestFlight ahead of the
App Store release.

Requires macOS 14+ (Apple Silicon or Intel) and USB tethering enabled on an RNDIS
device. Supported interface classes are 224/1/3, 2/2/255 and 239/4/1; non-composite
224/0/0 devices are unsupported. Released builds work only with devices from the 25
vendors in Apple's DriverKit grant.

## Source

The driver extension is open source. Its source is published at
[github.com/alessionossa/RNDISaurus](https://github.com/alessionossa/RNDISaurus), one
snapshot per released build: tag `v<version>-<build>` matches the version shown in the
app's About window. The app itself is closed source.

- `RNDISaurusDriver/` — the DriverKit extension: RNDIS protocol (`RNDISProtocol.*`, no
  DriverKit dependency), control plane, data path, diagnostics user client,
  `Info.plist` matching personalities and the development-signing entitlements.
- `Packages/RNDISaurusDiagnostics/Sources/RNDISaurusDiagnosticsC/include/RNDISaurusDiagnostics.h`
  — the plain-C diagnostics snapshot the driver returns to its clients.

These sources are not a standalone build: a dext is built as a DriverKit target of an
Xcode project, and runs only when signed with DriverKit entitlements granted by Apple.

## License

The driver is licensed under the [Mozilla Public License 2.0](LICENSE) and the
diagnostics header under the MIT License; everything else is proprietary. Details and
third-party notices are in [NOTICE.md](NOTICE.md).

## Acknowledgements

RNDISaurus is a successor in spirit to [HoRNDIS](https://github.com/jwise/HoRNDIS) by
Joshua Wise. Its behaviour was also informed by the Linux RNDIS host and gadget drivers
by [David Brownell](http://lwn.net/Articles/437618/),
[JellyBrick/ReRNDIS](https://github.com/JellyBrick/ReRNDIS),
[prostec-labs/TetherKit](https://github.com/prostec-labs/TetherKit) and
[XiaoMiku01/TetherKit](https://github.com/XiaoMiku01/TetherKit).

## Issues

Bug and device reports are welcome as
[issues](https://github.com/alessionossa/RNDISaurus/issues). For a device problem,
include the phone model, its Android version, and the report from **Window ›
Diagnostics › Copy Diagnostics** in the app.

The published source is a snapshot of each release: pull requests are not accepted, and
code pasted into issues is not merged. Fixes are written and released through the app.
