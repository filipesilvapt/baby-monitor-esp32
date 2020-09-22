# Baby Monitor (ESP32)
Monitor that checks the temperature and agitation levels of a baby and sends the information to Firebase.

## Notes:
To allow the definition of custom pins on the I2C connection of the accelerometer MLX90614 it was required to add a new begin function to its library.

- Edit the file Adafruit_MLX90614.h and add the new function signature to it:
```
bool begin(int sda, int scl);
```
- Edit the file Adafruit_MLX90614.cpp and add the new function to it:
```
bool Adafruit_MLX90614::begin(int sda, int scl) {
  Wire.begin(sda, scl);
  return true;
}
```
## Material:
- Thermometer: MLX90614
- Accelerometer: LIS3DH
- ESP32: Wemons lolin 32
----------------------------------------------------------------

## License
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
