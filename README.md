# Teensy 4.1-based Maze Robot code // RoboCup Junior 2023

To compile and upload + serial monitor:

``` bash
pio run --target upload && pio device monitor
```

or

``` bash
pio run --target upload && sleep 1 && pio device monitor
```

if the device monitor doesn't pick up the correct serial port

