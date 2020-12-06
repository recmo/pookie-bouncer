# Pookie bouncer

**Goal**: Control a NEMA17 servo using four potentiometers that control:

* Position: Direct position control.
* Velocity: Set a constant CW or CCW rate of rotation.
* Frequency: Set frequency of sinusoidal movement.
* Amplitude: Set amplitude of sinusoidal movement.

All control should obey a maximum [jerk](https://en.wikipedia.org/wiki/Jerk_%28physics%29).

The total continous movement is the sum of the velocity and the sinusoidal movement. Added to this
is any sporadic movement to satisfy position requests within the jerk limit.

**Note.** The Heltec Arduino board is currently broken on Mac OS Big Sur. See
[this issue](https://github.com/espressif/arduino-esp32/issues/4408). To work around
download a fixed [esptool](https://github.com/espressif/arduino-esp32/files/5556528/esptool.zip)
and place it in `~/Library/Arduino15/packages/Heltec-esp32/tools/esptool_py/2.6.1`, overwriting
the existing. Then convince mac to execute it.

## References

* Heltec WiFi Kit 32
  * [vendor](<https://heltec.org/project/wifi-kit-32/>)
  * [docs](https://heltec-automation-docs.readthedocs.io/en/latest/esp32+arduino/index.html)
  * [pinout (pdf)](https://resource.heltec.cn/download/WiFi_Kit_32/WIFI%20Kit%2032_pinoutDiagram_V1.pdf)
  * [Arduino IDE install guide](https://heltec.org/wifi_kit_install/)
  * [source](https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/tree/master/esp32)
  * [oled/lora source](https://github.com/HelTecAutomation/Heltec_ESP32)
  * [oled api doc](https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/src/oled/API.md)
* [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
  * [API docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference)
  * [hal source](https://github.com/espressif/esp-idf)
* BIQU A4988 Compatible motor driver modules
   * [vendor](https://www.biqu.equipment/products/1pcs-3d-printer-kit-a4988-stepper-motor-driver-module-with-heatsinks-reprap-board-for-3d-printer-free-shipping)
   * [A4988 datasheet](https://www.pololu.com/file/0J450/A4988.pdf)
   * [module pinout](https://www.pololu.com/product/1182)
* NEMA17 Stepper 17HS19-2004S1
  * [vendor](https://www.omc-stepperonline.com/nema-17-stepper-motor/nema-17-bipolar-59ncm-84oz-in-2a-42x48mm-4-wires-w-1m-cable-and-connector.html?mfp=146-rated-current-a%5B2.00%2C2%2C2.0%2C2.10%2C2.1%2C2.3%5D)
  * [datasheet](https://www.omc-stepperonline.com/download/17HS19-2004S1.pdf)
  * [torque curve](https://www.omc-stepperonline.com/download/17HS19-2004S1_Torque_Curve.pdf)
* [ESP32 based stepper controller](https://github.com/bdring/Grbl_Esp32)
* [Simple Kalman Filter library](https://github.com/denyssene/SimpleKalmanFilter)
* [Sampling ADC from interupt](https://www.toptal.com/embedded/esp32-audio-sampling)
* [S curves based motion planning](https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article)
  * Takeaway: For stepper motors the torque and hence maximum acceleration is a function of the current velocity.