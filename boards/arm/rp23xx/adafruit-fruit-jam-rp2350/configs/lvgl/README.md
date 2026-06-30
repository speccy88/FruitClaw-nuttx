# Adafruit Fruit Jam RP2350 LVGL

This profile runs LVGL through the standard NuttX framebuffer backend.

Expected display device:

- `/dev/fb0`
- 320x240 framebuffer
- RGB565, 16 bits per pixel
- 640-byte stride
- DVI/HSTX output at 640x480 with 2x scaling

Build from the NuttX tree with the sibling apps checkout:

```sh
./tools/configure.sh -E -m -a ../apps adafruit-fruit-jam-rp2350:lvgl
make -j$(sysctl -n hw.ncpu)
```

Flash only when the board is in BOOTSEL:

```sh
picotool info -a
picotool load -x nuttx.uf2
```

From NSH:

```sh
ls /dev
dvictrl info
dvictrl pattern colorbars
dvictrl start
lvgldemo widgets
```

`lvgldemo` should use `/dev/fb0` through `CONFIG_LV_USE_NUTTX`.
Do not enable `CONFIG_LV_USE_NUTTX_LCD` for this board unless a real
`/dev/lcd0` device is added later.
