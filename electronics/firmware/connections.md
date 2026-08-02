esp32:

&#x20; board: Seeed XIAO ESP32-S3 Sense



power:

&#x20; "ESP32 3V3":

&#x20;   - "BMP580 VIN"

&#x20;   - "LSM6DSO32 VIN"



&#x20; "ESP32 GND":

&#x20;   - "BMP580 GND"

&#x20;   - "LSM6DSO32 GND"



i2c:

&#x20; "ESP32 D4 (GPIO5 / SDA)":

&#x20;   - "BMP580 SDA"

&#x20;   - "LSM6DSO32 SDA"



&#x20; "ESP32 D5 (GPIO6 / SCL)":

&#x20;   - "BMP580 SCL"

&#x20;   - "LSM6DSO32 SCL"



battery:

&#x20; "Battery +":

&#x20;   - "Switch COM"



&#x20; "Switch NO":

&#x20;   - "ESP32 BAT+"



&#x20; "Battery -":

&#x20;   - "ESP32 BAT-"



unused:

&#x20; - "BMP580 3Vo"

&#x20; - "BMP580 CS"

&#x20; - "BMP580 SDO"

&#x20; - "LSM6DSO32 INT1"

&#x20; - "LSM6DSO32 INT2"

&#x20; - "LSM6DSO32 CS"

&#x20; - "LSM6DSO32 SDO"

