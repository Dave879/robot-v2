# Teensy 4.1-based Maze robot code

To compile and upload + serial monitor:

``` bash
pio run --target upload && pio device monitor
```

or

``` bash
pio run --target upload && sleep 1 && pio device monitor
```

if the device monitor doesn't pick up the correct serial port

## TODO

1. Refactoring
1. Fix data_formatter for ArduPlot
1. OpenMV communication

## Done

1. Speed up TCS34725 readings
1. Tested VL53L5CX (Chosen)
1. Tested TCS34725 sensor (Chosen)
1. Tested MPU6050 (Chosen)
1. Tested VL52L1X sensors
1. Tested ICM-20948
