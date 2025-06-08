# ESP32-c6 Bi Copter

This project is a work in progress.

Inspired by Nicholas Drehm's project. Some of the code is also taken from there.

The Coreless 8520 motors are not strong enough, 2x coreless 8520 motors are not able to lift off.
I got a new 400mAh 40C batter will test to see if it lifts off or the si2300 MOSFETs will burns.

If it still does not work I will turn it into a quadcopter.

## Components
* esp32-c6 supermini
* drv8833 dual H-bridge for actuator control, I can use it for controlling motors as well if I convert it into quadcopter.
* Two si2300 MOSFETs for driving coreless motors.
* actuators again vertically hanging actuators are also not strong, I added 10Ohms Resistor for precaution of overcurrent. But even if I remove that I think it wont be strong.
* TBD Two micro 2g servos
* TBD 1104 brushless motor with 1s battery 4A ESC. Actually some aliexpress pages say its 6A which would be ideal but others say its 4A so I am hoping it does 6A.

Weight - again no idea should be 40gram or so.

## Image

![bicopter1](docs-img/bicopter1.jpg)
![bicopter2](docs-img/bicopter2.jpg)

Some empty slots to convert to quad
![empty-slots-forQuad](docs-img/empty-slots-forQuad.jpg)