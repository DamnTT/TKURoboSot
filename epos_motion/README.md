## System Setting

Install MAXON SDK Library to system
```bash
$ cd <fira_ws>/src/epos_motion/epos_lib
$ sudo ./install.sh
# Check /usr/lib/libEposCmd.so /usr/lib/libftd2xx.so /etc/udev /etc/udev/rules.d/
```

## Connect Setting

Computer <- USB Cable -> Controller 1 <- CAN2CAN Cable -> Controller 2 <- CAN2CAN Cable -> Controller 3
                            |                                 |                                |
                          Motor 1                           Motor 2                          Motor 3
(Motor include 10 to 5 Encoder Cable, and 4 to 2 pins Power Cable)