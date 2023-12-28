*****************************************************************************
** ChibiOS/RT port for x86 into a Posix process                            **
*****************************************************************************

** TARGET **

The demo runs under any Posix IA32 system as an application program. The serial
I/O is simulated over TTY device.

** The Demo **

A thread is started that serves a small command shell. The command shell can be
exited with the exit command. A new command shell thread is then spawned.  The
demo shows how to create/terminate threads at runtime, how to listen to events,
how to work with serial ports, how to use the messages.  You can develop your
ChibiOS/RT application using this demo as a simulator then you can recompile it
for a different architecture.  See demo.c for details.

** Build Procedure **

The demo was built using GCC.

** Connect to the demo **

1. Create a PTY pair using socat

  $ socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
  2023/12/27 21:19:17 socat[539] N PTY is /dev/pts/2
  2023/12/27 21:19:17 socat[539] N PTY is /dev/pts/3
  2023/12/27 21:19:17 socat[539] N starting data transfer loop with FDs [6,6] and [9,9]

2. Use screen to connect to one end of the PTY pair.

  $ screen /dev/pts/3 # may have to use sudo

3. In another terminal, use the other end of the PTY pair as a command line
argument to the demo.

  $ ch /dev/pts/2

4. You should see the following in the screen program (below 'help' was entered
at the first prompt).

  ChibiOS/RT Shell
  ch> help
  Commands: help exit info echo systime mem threads test
  ch>
