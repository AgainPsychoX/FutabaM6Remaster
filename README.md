
# Futaba M series 6 remaster



<!----------------------------------------------------------------------------->



## Hardware

<!-- pins -->


<!----------------------------------------------------------------------------->

## Software

+ Transmitter reads state from the controls via potentiometers, using analog inputs. The values are normalized and transformed to precalculated values for receiver use, like number of microseconds to control the servos. That way the receiver doesn't need to be configured - at least for now.



<!----------------------------------------------------------------------------->

## Notes

Interesting:
+ https://community.platformio.org/t/new-esp32-s3-n16r8/41568/4
+ https://electronoobs.com/eng_robotica_tut4_2.php
+ https://github.com/atomic14/esp32-s3-pinouts

### To do

1. pages:
	1. transmitter battery, signal strength, receiver battery
	2. raw analog values (done)
	3. centered analog values, long press to remember current values as 0.
	4. auto-calibration for raw analog values - move controls around, getting min/max values, to be used as 0% & 100%
	5. manual calibration
		+ move controls to select channel; viewing its name and 6 values.
		+ then change with aileron/elevator
		+ press F1 to save
	6. reverse configuration - move controls to select, long press F1 to switch

+ "Hello" packet: transmitter asking for receiver, receiver accepts.
+ Update README to be actually useful and nice.
+ Get rid of warnings from 3rd party code.
