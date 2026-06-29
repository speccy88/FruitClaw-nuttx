# GPIO29 User LED Archive

GPIO29 on the Adafruit Fruit Jam is shared by the active-low red user LED and
the IR receiver.  The `esp-hosted` NuttX configuration now gives this pin to
the IR receiver and registers it as `/dev/lirc0`.

The old user LED scripts are kept here for reference only.  They are not baked
into `/examples`, and `CONFIG_USERLED` is disabled for this image so shell
commands cannot accidentally drive the IR receiver pin as an output.
