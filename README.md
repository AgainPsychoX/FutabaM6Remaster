
# Futaba M series 6 remaster



<!----------------------------------------------------------------------------->



## Hardware

<!-- pins -->


<!----------------------------------------------------------------------------->

## Software



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
	4. calibration - move controls around, getting min/max values, to be used as 0% & 100% when generating PPM on the receiver.
	5. reverse configuration - move controls to select, long press F1 to switch

+ "Hello" packet: transmitter asking for receiver, receiver accepts.
+ Configure the board properly https://docs.platformio.org/en/latest/platforms/creating_board.html
+ Read https://community.platformio.org/t/how-to-structure-project-with-multiple-boards-with-different-tasks/13287/2
+ Get rid of warnings from 3rd party code.
