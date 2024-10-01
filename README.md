
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

1. Fix throttle - allow zero being at min... or rather: there is no center value for it.
2. Reduce blinking at `Page::Centered`
3. Rework `Page::Reverse`: use right stick
4. Hide some pages, unless starting with F1 pressed
5. Reset to known values when starting with F1 pressed and some joystick state.
6. Update README

+ "Hello" packet: transmitter asking for receiver, receiver accepts.
+ Update README to be actually useful and nice.
+ Get rid of warnings from 3rd party code.
