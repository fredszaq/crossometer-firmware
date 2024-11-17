# crossometer-firmware

crossometer-firmware is a firmware for GPS/altimeter/variometer devices. 

The main target is the [crossometer-mini](https://github.com/fredszaq/crossometer-mini) board, but any esp32 board connected to the proper sensors should work.

## Current features
 - barometer (bmp280) based naive variometer algorithm with sound output
 - altimeter (both using GPS and barometer, the first GPS point is used to calibrate the barometer)
 - OLED screen support with altitude, speed, vertical speed, finesse and time


## Planned features
 - kalman filter aggregating barometer gps and accelerometer
 - sdcard support with gpx traces
 - webserver mode with trace visualizer and configuration when not flying


## Licence

crossometer-firmware is released under the GPLv3 licence


