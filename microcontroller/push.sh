sudo chmod a+rw /dev/ttyUSB0
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32-poe-iso modular_controller/
