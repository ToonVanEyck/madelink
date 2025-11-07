# madelink 🌼🔗

![Madelink Logo](madelink-logo.svg)

Madelink is a portmanteau of "madeliefje" (Dutch for daisy) and "link" (part of a chain). Madelink is a daisy-chainable master-slave protocol using serial communication (UART) for connecting multiple microcontroller-based devices.

The protocol is originally designed as part of the [OpenFlap](https://github.com/ToonVanEyck/OpenFlap) project to connect a single controller to multiple nodes over a single serial bus. As such the the master is implemented to be used as part of an RTOS-based system, while the slaves are implemented to run on bare-metal microcontrollers.

## Features
- Daisy chain
- Master-slave protocol
- Read / Write / Broadcast support
- Error checking
- Dynamic data lengths 
- No dynamic memory allocation
- Automatic chain length detection