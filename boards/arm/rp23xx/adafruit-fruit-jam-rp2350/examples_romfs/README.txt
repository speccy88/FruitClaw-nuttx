Fruit Jam NuttX examples
========================

Run these with source, for example:

  source /examples/neopixels_rainbow.sh
  source /examples/neopixels_fire.sh
  source /examples/neopixels_off.sh
  source /examples/rtttl_gta3.sh
  source /examples/rtttl_testtone.sh

Direct commands:

  neopixels rgb 255 0 0 64
  neopixels red 64
  neopixels green 64
  neopixels blue 64
  neopixels rainbow 1 15 96
  neopixels fire 80 40 128
  neopixels chase blue 8 80 96
  neopixels pulse purple 6 35 128
  rtttl -r 22050 -v 90 gta3
  rtttl -r 22050 -v 90 scratchy
  rtttl -r 22050 -v 90 simpsons
  rtttl -r 22050 -v 90 cantina
  rtttl -l
  rtttl -r 22050 -v 95 "testtone:d=1,o=4,b=120:a,a,a,a"
  rtttl -r 22050 -v 90 "gta3:d=4,o=5,b=100:8b,8b,32b,16p,32p,16a,16b,16d6,16b,16p,16a,8b."

Variable-based helpers:

  set NEO_R 255
  set NEO_G 0
  set NEO_B 0
  set NEO_BRIGHTNESS 64
  source /examples/neopixels_color.sh

RTTTL helpers:

  source /examples/rtttl_gta3.sh
  source /examples/rtttl_scratchy.sh
  source /examples/rtttl_simpsons.sh
  source /examples/rtttl_cantina.sh
  source /examples/rtttl_testtone.sh
  source /examples/rtttl_list.sh

NSH variable expansion here uses $NAME form, not ${NAME}.

This /examples directory is baked into the firmware image as ROMFS and is
read-only. Use /scripts for temporary writable scripts.
