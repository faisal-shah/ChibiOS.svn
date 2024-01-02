*****************************************************************************
** ChibiOS/RT port for x86 into a Posix process                            **
*****************************************************************************

** TARGET **

The demo runs under Linux with socketCAN support as an application program.

** The Demo **

The demo sniffs the socketCAN bus and prints out the CAN frames.
It also transmits CAN frames with incrementing data every second with ID
0x80085.

This demonstrates the usage of the simulator CAN driver.

** Build Procedure **

The demo was built using GCC.

** Sample output **

$ # Create virtual CAN device (vcan0)
$ sudo ip link add type vcan vcan0
$ # Activate vcan0 device
$ sudo ip link set dev vcan0 up
$ # generate CAN frames periodically and put it in the background
$ cangen vcan0 &
$ ./build/ch vcan0
ChibiOS/RT simulator (Linux)

(002.826174) 000001CF#6FBBD96BFD02E84D
(003.028118) 000002BC#95853C
(003.228187) 000002B7#7E
(003.428095) 000003E7#7AE1C05AB8F3B04D
(003.628094) 000003A7#14D70A
(003.828155) 000005B3#F8
(004.029206) 0000039F#D020272D46CE0F42
(004.229184) 00000505#F82E4B32D5AEA9
(004.429135) 00000094#D2B1911651B372
(004.629132) 0000055B#A1676D5D126010
(004.829096) 00000168#FDD12000509DFC7C
(005.029097) 0000053F#0891AD4AE60813
(005.230131) 0000059B#709CF4744EF32A19
(005.430101) 0000031E#6814B86DBDEED878
