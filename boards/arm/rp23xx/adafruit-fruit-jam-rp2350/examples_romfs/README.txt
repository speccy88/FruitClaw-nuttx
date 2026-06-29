Fruit Jam NuttX examples
========================

Run these with source, for example:

  source /examples/neopixels_rainbow.sh
  source /examples/neopixels_fire.sh
  source /examples/neopixels_off.sh

Direct commands:

  neopixels rgb 255 0 0 64
  neopixels red 64
  neopixels green 64
  neopixels blue 64
  neopixels rainbow 1 15 96
  neopixels fire 80 40 128
  neopixels chase blue 8 80 96
  neopixels pulse purple 6 35 128

Variable-based helpers:

  set NEO_R 255
  set NEO_G 0
  set NEO_B 0
  set NEO_BRIGHTNESS 64
  source /examples/neopixels_color.sh

NSH variable expansion here uses $NAME form, not ${NAME}.

This /examples directory is baked into the firmware image as ROMFS and is
read-only. Use /scripts for temporary writable scripts.
