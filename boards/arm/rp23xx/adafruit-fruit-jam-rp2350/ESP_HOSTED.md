# Fruit Jam ESP-Hosted Wi-Fi Bring-Up

This note tracks the raw-frame ESP-Hosted path for the Adafruit Fruit Jam
RP2350 board. The goal is a real NuttX `wlan0` interface backed by the onboard
ESP32-C6 coprocessor. This is not the Adafruit NINA/AirLift socket-command
firmware path.

## Target Shape

- RP2350 runs NuttX.
- ESP32-C6 runs ESP-Hosted-MCU slave/coproc firmware.
- NuttX owns IP addressing, DHCP, DNS, sockets, and services.
- ESP32-C6 handles Wi-Fi radio work and exchanges control/data frames with the
  RP2350 host.
- User-space sees normal NuttX networking: `ifconfig wlan0`, WAPI/wireless
  ioctls, DHCP, DNS, ping, TCP/UDP sockets, telnet, HTTP, and FTP.

## Current Code State

The current tree contains an ESP-Hosted SPI scaffold:

- `include/nuttx/wireless/esp_hosted.h`
- `drivers/wireless/esp_hosted.c`
- `drivers/wireless/Kconfig`
- `drivers/wireless/Make.defs`
- `drivers/wireless/CMakeLists.txt`
- `boards/arm/rp23xx/adafruit-fruit-jam-rp2350/Kconfig`
- `boards/arm/rp23xx/adafruit-fruit-jam-rp2350/src/rp23xx_bringup.c`

The board option is `CONFIG_ADAFRUIT_FRUIT_JAM_RP2350_ESP_HOSTED`. It depends
on `CONFIG_ESP_HOSTED_SPI` and `CONFIG_RP23XX_SPI1`, and selects the board's
onboard ESP32-C6 pin setup.

The scaffold validates the configured SPI/GPIO callbacks, attaches the
data-ready IRQ when the board provides one, resets the coprocessor, configures
SPI, and uses 1600-byte ESP-Hosted full-duplex frames. The host now sends a
valid dummy header (`ESP_MAX_IF`) instead of an all-zero buffer, schedules
data-ready service on `HPWORK`, validates incoming hosted headers and
checksums, counts private/control/station/AP frames, and recognizes
`ESP_PRIV_EVENT_INIT`.

After `ESP_PRIV_EVENT_INIT`, the driver now sends two minimal ESP-Hosted RPC
startup probes over the serial/control interface:

- `Req_GetCoprocessorFwVersion` (`350`) with an empty request payload.
- `Req_GetMACAddress` (`257`) with station Wi-Fi interface mode `0`.

The parser recognizes `Resp_GetCoprocessorFwVersion` (`606`) and
`Resp_GetMACAddress` (`513`), records RPC counters, and logs the firmware
version / IDF target / chip ID and station MAC when the coprocessor returns
valid responses.

The driver now contains the first real NuttX lower-half plumbing for the
station data path. It allocates a `netdev_lowerhalf_s`, registers it as
`NET_LL_IEEE80211` only after both identity RPCs succeed, queues incoming
`ESP_STA_IF` frames into NuttX `netpkt` RX buffers, and wraps NuttX TX packets
as `ESP_STA_IF` frames for the full-duplex SPI transport.

The station control path is now partially mapped to ESP-Hosted RPCs. After the
version/MAC identity probe succeeds, the driver sends:

- `Req_WifiInit` (`278`) with ESP32-C6/ESP-IDF 5.5-style init defaults.
- `Req_SetWifiMode` (`260`) for station mode.
- `Req_WifiStart` (`280`).

WAPI/wireless ioctl state for ESSID, passphrase, and WPA/cipher preferences is
kept in driver RAM. A connect request sends `Req_WifiSetConfig` (`284`) with
the configured station SSID/passphrase and then `Req_WifiConnect` (`282`).
Disconnect sends `Req_WifiDisconnect` (`283`). Scan start sends
`Req_WifiScanStart` (`286`). `Event_StaScanDone` (`774`) is decoded, and the
first `SIOCGIWSCAN` after completion fetches up to 12 AP records with
`Req_WifiScanGetApRecords` (`289`) and formats them as NuttX `iw_event`
records for WAPI. Carrier is turned on/off only from real
`Event_StaConnected` (`775`) and `Event_StaDisconnected` (`776`)
notifications from the coprocessor.

## RP2350-Side Pins

These pins are the existing Fruit Jam NINA/AirLift-style wiring on the RP2350
side. They are used only as board wiring for the ESP-Hosted path.

| Signal | RP2350 GPIO | NuttX symbol |
| --- | ---: | --- |
| SPI1 SCK | 30 | `BOARD_NINA_SCK_PIN` |
| SPI1 MOSI | 31 | `BOARD_NINA_MOSI_PIN` |
| SPI1 MISO | 28 | `BOARD_NINA_MISO_PIN` |
| SPI1 CS | 46 | `BOARD_NINA_CS_PIN` |
| ESP data-ready IRQ | 23 | `BOARD_NINA_IRQ_PIN` |
| ESP handshake/busy | 3 | `BOARD_NINA_READY_PIN` |
| ESP/peripheral reset | 22 | `BOARD_NINA_RESET_PIN` |

GPIO22 is also used by other Fruit Jam peripheral reset wiring. Keep reset
sequencing conservative while bring-up is in progress.

## ESP32-C6-Side Pins

The ESP32-C6-side pins have now been checked against the official Adafruit
Fruit Jam Eagle schematic from `adafruit/Adafruit-Fruit-Jam-PCB` commit
`252dc09456ee447b3d7eeb4fa99877898503e81a`. The RP2350-side names were also
cross-checked against the CircuitPython Fruit Jam board definition at commit
`44171479b70f0cf71e09a68d89507bfb8657057f`.

| ESP-Hosted signal | RP2350 side | ESP32-C6 side | Schematic net |
| --- | --- | --- | --- |
| SCK | GPIO30 | IO22 | `SCK` |
| MOSI, host to C6 | GPIO31 | IO21 | `MOSI` |
| MISO, C6 to host | GPIO28 | IO6 through `ESP_MISO` buffer | `MISO` |
| CS | GPIO46 | IO7 | `ESP_CS` |
| Handshake/busy | GPIO3 | IO18 | `ESP_BUSY` |
| Data-ready IRQ | GPIO23 | IO9 / BOOT9 | `I2S_ESP_IRQ` |
| Reset | GPIO22 | EN | `PERIPH_RST` |

`I2S_ESP_IRQ` is also the ESP32-C6 BOOT9 strap. Keep the RP2350 side as a
high-impedance input through ESP32-C6 reset/boot; the ESP-Hosted slave firmware
can drive it as data-ready only after startup.

The matching ESP-Hosted-MCU slave Kconfig overlay is:

- `esp-hosted-mcu/sdkconfig.defaults.fruitjam-esp32c6`

## ESP32-C6 Firmware Notes

Preferred upstream starting point:

- `https://github.com/espressif/esp-hosted-mcu`
- `https://components.espressif.com/components/espressif/esp_hosted`

Initial firmware flow:

```sh
git clone --recurse-submodules https://github.com/espressif/esp-hosted-mcu.git
cd esp-hosted-mcu/slave
cp /Users/fred/Documents/FruitClaw/nuttx/boards/arm/rp23xx/adafruit-fruit-jam-rp2350/esp-hosted-mcu/sdkconfig.defaults.fruitjam-esp32c6 .
. /Users/fred/esp/v5.5.4/esp-idf/export.sh
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32c6;sdkconfig.defaults.fruitjam-esp32c6" set-target esp32c6
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32c6;sdkconfig.defaults.fruitjam-esp32c6" build
idf.py merge-bin
```

The overlay chooses SPI full-duplex transport, mode 3, and the Fruit Jam
ESP32-C6 GPIOs listed above. Start the RP2350 host at a low SPI clock such as
5 MHz and raise it only after error counters and signal behavior are understood.

Flash through the RP2350 serial-passthrough UF2:

```sh
python -m esptool --chip esp32c6 --before no_reset --after no_reset \
  -p /dev/cu.usbmodem<PASSTHROUGH> -b 115200 \
  write_flash 0 build/merged-binary.bin
```

The known local passthrough UF2 is:

```text
/Users/fred/Documents/Code/PebbleRP2350/hardware_tests/fruitjam_esp32c6_hci/tools/SerialESPPassthrough.ino.uf2
```

After flashing the ESP32-C6, reflash the RP2350 NuttX UF2 before testing
`wlan0`.

## NuttX Configuration Notes

Keep the existing `adafruit-fruit-jam-rp2350:usbnsh` profile stable while the
driver is still being validated. The named
`adafruit-fruit-jam-rp2350:esp-hosted` profile carries the current experimental
network configuration. If rebuilding it manually, keep at least:

```text
CONFIG_NET=y
CONFIG_NET_ETHERNET=y
CONFIG_NET_IPv4=y
CONFIG_NET_TCP=y
CONFIG_NET_UDP=y
CONFIG_NET_ICMP=y
CONFIG_NET_ARP=y
CONFIG_NETDEVICES=y
CONFIG_NETDEV_LATEINIT=y
CONFIG_DRIVERS_WIRELESS=y
CONFIG_ESP_HOSTED=y
CONFIG_ESP_HOSTED_SPI=y
CONFIG_ESP_HOSTED_WLAN=y
CONFIG_ADAFRUIT_FRUIT_JAM_RP2350_ESP_HOSTED=y
CONFIG_NETDEV_WIRELESS_IOCTL=y
CONFIG_NETDEV_WIRELESS_HANDLER=y
CONFIG_DRIVERS_IEEE80211=y
CONFIG_NETUTILS_DHCPC=y
CONFIG_NETDB_DNSCLIENT=y
CONFIG_SYSTEM_PING=y
```

The profile also includes WAPI tooling, DHCP, DNS, `ping`, `wget`, and
`telnetd` so early link tests can run from NSH. HTTP/FTP servers and broader
socket tests can be added after basic `wlan0` traffic is proven.

## Build Evidence

Built locally on 2026-06-29:

| Profile | Result | FLASH | RAM | Notes |
| --- | --- | ---: | ---: | --- |
| `adafruit-fruit-jam-rp2350:usbnsh` | Pass | 406576 B | 51060 B | Baseline image after returning from hosted config |
| `adafruit-fruit-jam-rp2350:esp-hosted` | Pass | 489080 B | 70016 B | Named defconfig with `CONFIG_ESP_HOSTED_WLAN=y`, WAPI, DHCP, DNS, `ping`, `wget`, and `telnetd` compiled |
| ESP32-C6 ESP-Hosted-MCU slave | Pass | 1026464 B | n/a | Upstream `esp-hosted-mcu` commit `8f0770d39065c2a9ff6828268709c3502e0d5349`, ESP-IDF 5.5.4, Fruit Jam defaults overlay, `merged-binary.bin` SHA256 `a7e78def271dc2a21c6983ce2dd5883b7e95a7c6576fae263edda82c5dceec6c` |

## Validation Milestones

| ID | Milestone | Current state | Evidence needed |
| --- | --- | --- | --- |
| A | ESP32-C6 reset works from RP2350 | Partial | Schematic maps RP2350 GPIO22/PERIPH_RST to C6 EN; need scope trace or ESP firmware log after NuttX reset callback |
| B | SPI exchanges valid ESP-Hosted control frames | In progress | Fruit Jam C6 SPI pins are mapped and a slave defaults file exists; host still needs valid ESP-Hosted INIT or RPC response |
| C | NuttX gets coprocessor version/MAC | In progress | `GetCoprocessorFwVersion` and `GetMacAddress` responses in NuttX log |
| D | Scan returns AP records | In progress | `WifiScanStart`, `Event_StaScanDone`, `WifiScanGetApRecords`, and WAPI result formatting are implemented; hardware validation still needed |
| E | Connect and carrier event | In progress | `WifiSetConfig`/`WifiConnect` requests and STA connected/disconnected carrier handling are implemented |
| F | `wlan0` appears | In progress | `ifconfig wlan0` shows a registered netdev after version/MAC RPCs |
| G | DHCP through NuttX | Not started | NuttX DHCP client assigns `wlan0` IPv4 address |
| H | Ping works | Not started | `ping <gateway>` succeeds through `wlan0` |
| I | TCP client works | Not started | NuttX TCP client or `wget` succeeds |
| J | Inbound service works | Not started | telnet, HTTP, or FTP service reachable over `wlan0` |
| K | Reconnect/reset recovery works | Not started | Reset or AP reconnect returns `wlan0` to service |

## Bring-Up Commands

Baseline known-good build:

```sh
cd /Users/fred/Documents/FruitClaw/nuttx
./tools/configure.sh -E -m -a ../apps adafruit-fruit-jam-rp2350:usbnsh
make -j8
```

Experimental ESP-Hosted build:

```sh
cd /Users/fred/Documents/FruitClaw/nuttx
./tools/configure.sh -E -m -a ../apps adafruit-fruit-jam-rp2350:esp-hosted
make -j8
```

Runtime validation commands once `wlan0` is implemented:

```sh
ifconfig
ifconfig wlan0
wapi scan wlan0
ifconfig wlan0 dhcp
ping <gateway-ip>
wget http://<test-host>/
```

## Near-Term Driver Tasks

1. Validate the initial `WifiInit`/`SetWifiMode`/`WifiStart` sequence against
   a Fruit Jam ESP32-C6 running ESP-Hosted-MCU firmware.
2. Validate `wapi scan wlan0` against ESP-Hosted-MCU firmware, then expand AP
   record fetching beyond the current conservative 12-record SPI-frame batch.
3. Add RSSI/link-state/MAC query support where NuttX wireless tooling expects
   it.
4. Verify that received `ESP_STA_IF` payloads are in the exact link-layer
   format expected by the registered NuttX netdev type.
5. Bring up DHCP, DNS, ping, and TCP/UDP service tests through `wlan0`.
6. Build and flash the ESP-Hosted-MCU Fruit Jam C6 firmware with the local
   defaults overlay, then validate the first `ESP_PRIV_EVENT_INIT` handshake.
