Overview
========
The CAN-FD msgobjs project is a demonstration program that uses the KSDK software to send CAN-FD
messages from CAN0 to CAN1 using message buffers.

The example toggles LED1 every time an 11-bit message is transmitted, toggles LED2 every time an
11-bit message is received, and toggles LED3 every time a 29-bit message is received.

Messages are transmitted from CAN0 every 100ms using dedicated message buffers. Messages are transmitted
with 64 bytes of data at 1Mbps nominal bitrate and 4Mbps data bitrate with bitrate switching enabled.

Messages are received into dedicated message buffers. Several filters are configured to allow multiple
messages to be received into a single message buffer. A second message buffer is configured to receive
messages with a specific 29-bit identifier.

Toolchain supported
===================
- Keil MDK 5.21a
- MCUXpresso0.8

Hardware requirements
=====================
- Micro USB cable
- CAN cable
- LPCXpresso54618 CAN-FD kit (OM13094)
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
Connect a serial cable from the debug UART port of the board to the PC. Start Tera Term
(http://ttssh2.osdn.jp) and make a connection to the virtual serial port.

1. Start Tera Term
2. New connection -> Serial
3. Set appropriate COMx port (x is port number) in Port context menu. Number is provided by operation
   system and could be different from computer to computer. Select COM number related to virtual
   serial port. Confirm selected port by OK button.
4. Set following connection parameters in menu Setup->Serial port...
        Baud rate:    115200
        Data:         8
        Parity:       none
        Stop:         1
        Flow control: none
5.  Confirm selected parameters by OK button.
6. Attach the CAN shield to the LPCxpresso54618 board.
7. Connect both CAN connectors together using a CAN cable.

Running the demo
================
After the board is flashed the Tera Term will print message on terminal.

LEDs 1, 2 and 3 will toggle every 100ms.

Customization options
=====================

