# ESR+capacitance meter

This is an ATSAMD21G16x based ESR and capacitance meter. Due to program size,
at least the *16* chip is required (64 KB Flash, 8 KB RAM). The code is written so
that the meter automatically tests for both capacitance & ESR when a new capacitor
is connected. This meter can also be used as a low ohmage ohm-meter for resistors &lt;~100 ohms.
Currently the values are displayed on a 16x2 character display.


## Analog schematic:
![schematic](./Schematic/Capacitor%20ESR-meter+capacitance.svg)


## TODO

* [ ] _Nice_ schematic
* [ ] Photos of prototype
* [ ] (Maybe) utilize a better display in the future like 20x4
* [ ] Option to calibrate & adjust capacitance measurements
* [ ] Option to calibrate & adjust ESR measurements semi-automatically
* [ ] Should measure capacitance at least twice or wait to eliminate any contact bounce issues
* [ ] Calibration values via USB
* [ ] Option to save calibration values to flash via USB


## List of supported chips:

* [x] ATSAMD21E16A
* [x] ATSAMD21E16B
* [x] ATSAMD21G16A
* [x] ATSAMD21G16B
* [x] ATSAMD21J16A
* [x] ATSAMD21J16B

* [x] ATSAMD21E17A
* [x] ATSAMD21E17B
* [x] ATSAMD21G17A
* [x] ATSAMD21G17B
* [x] ATSAMD21J17A
* [x] ATSAMD21J17B

* [x] ATSAMD21E18A
* [x] ATSAMD21E18B
* [x] ATSAMD21G18A
* [x] ATSAMD21G18B
* [x] ATSAMD21J18A
* [x] ATSAMD21J18B

* [ ] ATSAMD20E16
* [ ] ATSAMD20G16
* [ ] ATSAMD20J16

* [ ] ATSAMD20E17
* [ ] ATSAMD20G17
* [ ] ATSAMD20J17

* [ ] ATSAMD20E18
* [ ] ATSAMD20G18
* [ ] ATSAMD20J18

* [ ] ATSAML21E16A
* [ ] ATSAML21E16B
* [ ] ATSAML21G16A
* [ ] ATSAML21G16B
* [ ] ATSAML21J16A
* [ ] ATSAML21J16B

* [ ] ATSAML21E17A
* [ ] ATSAML21E17B
* [ ] ATSAML21G17A
* [ ] ATSAML21G17B
* [ ] ATSAML21J17A
* [ ] ATSAML21J17B

* [ ] ATSAML21E18A
* [ ] ATSAML21E18B
* [ ] ATSAML21G18A
* [ ] ATSAML21G18B
* [ ] ATSAML21J18A
* [ ] ATSAML21J18B


### Future expansion:

* [ ] ATSAMC21E16
* [ ] ATSAMC21G16
* [ ] ATSAMC21J16

* [ ] ATSAMC21E17
* [ ] ATSAMC21G17
* [ ] ATSAMC21J17
* [ ] ATSAMC21N17

* [ ] ATSAMC21E18
* [ ] ATSAMC21G18
* [ ] ATSAMC21J18
* [ ] ATSAMC21N18


* [ ] ATSAMC20E16
* [ ] ATSAMC20G16
* [ ] ATSAMC20J16

* [ ] ATSAMC20E17
* [ ] ATSAMC20G17
* [ ] ATSAMC20J17
* [ ] ATSAMC20N17

* [ ] ATSAMC20E18
* [ ] ATSAMC20G18
* [ ] ATSAMC20J18
* [ ] ATSAMC20N18


## License

This project uses the MIT license.

