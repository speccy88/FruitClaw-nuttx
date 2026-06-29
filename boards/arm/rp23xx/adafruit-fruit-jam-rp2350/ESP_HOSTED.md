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
valid responses. This is still control-plane bring-up only; scan, connect, and
data-plane packet routing are not implemented yet.

It intentionally returns `-ENOSYS` and does not register `wlan0` until
ESP-Hosted RPC requests/responses and the data path are implemented. That
keeps the port from exposing a fake network interface before NuttX can really
own DHCP/IP/sockets.

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

Do not hardcode ESP32-C6 slave firmware GPIOs until they are confirmed from the
Fruit Jam schematic or board definition. ESP-Hosted-MCU exposes these settings
through the slave project Kconfig:

- `CONFIG_ESP_SPI_HSPI_GPIO_MOSI`
- `CONFIG_ESP_SPI_HSPI_GPIO_MISO`
- `CONFIG_ESP_SPI_HSPI_GPIO_CLK`
- `CONFIG_ESP_SPI_HSPI_GPIO_CS`
- `CONFIG_ESP_SPI_GPIO_HANDSHAKE`
- `CONFIG_ESP_SPI_GPIO_DATA_READY`
- `CONFIG_ESP_SPI_GPIO_RESET`

## ESP32-C6 Firmware Notes

Preferred upstream starting point:

- `https://github.com/espressif/esp-hosted-mcu`
- `https://components.espressif.com/components/espressif/esp_hosted`

Initial firmware flow:

```sh
git clone --recurse-submodules https://github.com/espressif/esp-hosted-mcu.git
cd esp-hosted-mcu/slave
idf.py set-target esp32c6
idf.py menuconfig
```

In menuconfig, choose SPI full-duplex transport and set the ESP32-C6-side pins
after schematic confirmation. ESP-Hosted full-duplex SPI uses MOSI, MISO, SCK,
CS, reset, handshake, and data-ready. Start with a low SPI clock such as
5 MHz and raise it only after error counters and signal behavior are understood.

Build and flash:

```sh
idf.py build
idf.py -p <esp32c6_serial_or_bridge_port> flash monitor
```

The exact Fruit Jam ESP32-C6 flashing path still needs to be verified. Do not
assume the RP2350 USB CDC NSH port can directly flash the ESP32-C6.

## NuttX Configuration Notes

Keep the existing `adafruit-fruit-jam-rp2350:usbnsh` profile stable while the
driver is scaffold-only. For the experimental ESP-Hosted profile, enable at
least:

```text
CONFIG_NET=y
CONFIG_NET_ETHERNET=y
CONFIG_NET_IPv4=y
CONFIG_NET_TCP=y
CONFIG_NET_UDP=y
CONFIG_NET_ICMP=y
CONFIG_NET_ARP=y
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
CONFIG_NETUTILS_DNSCLIENT=y
CONFIG_SYSTEM_PING=y
```

Additional app/service symbols should be added once `wlan0` is real:
`wapi`, `wget`, `telnetd`, `httpd`, `ftpd`, and socket tests.

## Validation Milestones

| ID | Milestone | Current state | Evidence needed |
| --- | --- | --- | --- |
| A | ESP32-C6 reset works from RP2350 | Partial | Scope trace or ESP firmware log after NuttX reset callback |
| B | SPI exchanges valid ESP-Hosted control frames | In progress | Host receives valid ESP-Hosted INIT or RPC response |
| C | NuttX gets coprocessor version/MAC | In progress | `GetCoprocessorFwVersion` and `GetMacAddress` responses in NuttX log |
| D | Scan returns AP records | Not started | `wapi scan wlan0` or equivalent returns AP list |
| E | Connect and carrier event | Not started | Connected event toggles NuttX carrier on |
| F | `wlan0` appears | Not started | `ifconfig wlan0` shows a registered netdev |
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

Experimental ESP-Hosted build once the network config is enabled:

```sh
cd /Users/fred/Documents/FruitClaw/nuttx
./tools/configure.sh -E -m -a ../apps adafruit-fruit-jam-rp2350:usbnsh
make menuconfig
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

1. Add RPC endpoint routing for `RPCRsp` and `RPCEvt`.
2. Add enough protobuf encode/decode support for `GetCoprocessorFwVersion`,
   `GetMacAddress`, `WifiInit`, and `WifiStart`.
3. Implement version/MAC RPC requests before registering `wlan0`.
4. Add `netdev_lowerhalf_s` registration with `NET_LL_IEEE80211` only after
   the control plane can identify the coprocessor.
5. Map NuttX wireless operations to ESP-Hosted RPCs for scan, connect,
   disconnect, RSSI, MAC, and link state.
6. Wrap NuttX TX packets as ESP-Hosted station data frames and deliver RX
   station frames into the NuttX stack.
