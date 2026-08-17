# RNDISaurus - modern USB tethering driver for macOS

*RNDISaurus* 🦖 brings RNDIS back from extinction on macOS allowing you to use your Android device's USB tethering functionality on macOS.

Modern Android phones, starting with Google Pixel devices, support CDC-NCM to allow native plug-and-play tethering on Linux and macOS without proprietary drivers, but many Android devices around are not compatible with this protocol.

Thanks to [HoRNDIS](https://github.com/jwise/horndis) for the inspiration and the original implementation. So, in turn thanks to David Brownell ([🙏🕊️](http://lwn.net/Articles/437618/)) who wrote `rndis_host` driver for Linux and `f_rndis` driver that allows Android/Linux devices to behave like RNDIS devices.

Apple [deprecated legacy kernel extensions](https://developer.apple.com/support/kernel-extensions/) (kexts) in macOS Catalina (10.15) in 2019 and stopped loading them by default in macOS Big Sur (11.0) in 2020, so [HoRNDIS](https://github.com/jwise/horndis) hasn't been an ideal solution for many years and its installation is [discouraged by its developer](https://www.joshuawise.com/horndis#stop-stop-stop), so there was the need of implementing this functionality with modern macOS APIs.