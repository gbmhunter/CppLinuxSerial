# CppLinuxSerial

Linux serial port library written in C++.

[![Build Status](https://travis-ci.org/gbmhunter/CppLinuxSerial.svg?branch=master)](https://travis-ci.org/gbmhunter/CppLinuxSerial)

## Description

Library for communicating with COM ports on a Linux system.

* Simple API
* Supports custom baud rates
* `cmake` based build system

## Installation

### Linux, MacOS, Windows

1. Make sure you have `cmake` installed.

1. Clone the git repo onto your local storage.

1. Change into root repo directory:

	```
	$ cd CppLinuxSerial
	```

1. Create a new build directory and change into it:

	```bash
	$ mkdir build
	$ cd build
	```

1. Run cmake on the parent directory to generate makefile:

	```bash
	$ cmake ..
	```

1. Run make on the generated makefile to generate the static library `libCppLinuxSerial.a` and an unit test executable:

	```bash
	$ make
	```

1. To install the headers on your system:

	```bash
	$ sudo make install
	```

1. To run the unit tests:

	```bash
	$ make run_unit_tests
	```

	NOTE: The unit tests used to use virtual serial ports via `stty` on Linux to do more through testing. I ran into permission problems running stty on TravisCI after they did an update and had to remove tests (leaving almost no tests remaining). If anyone wants to add better unit tests, it is greatly welcomed! 

## Examples

```c++
#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

int main() {
	// Create serial port object and open serial port at 57600 buad, 8 data bits, no parity bit, one stop bit (8n1),
	// and no flow control
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();

	// Write some ASCII data
	serialPort.Write("Hello");

	// Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
	std::string readData;
	serialPort.Read(readData);

	// Close the serial port
	serialPort.Close();
}
```

If the above code was in a file called `main.cpp` and you had installed `CppLinuxSerial` following the instructions above, on a Linux system you should be able to compile the example application with:

```bash
g++ main.cpp -lCppLinuxSerial
```

If you wanted to also set flow control (e.g. hardware), you can add it onto the end of the constructor as shown below. If you don't set it, it defaults to no flow control.

```c++
SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, FlowControl::HARDWARE);
```

If you want to read and write binary rather than strings, you can use `WriteBinary()` and `ReadBinary()` which take vectors of bytes rather than `std::string`:

```c++
serialPort.WriteBinary(const std::vector<uint8_t>& data);
serialPort.ReadBinary(std::vector<uint8_t>& data);
```

For more examples, see the files in `test/`.

## Issues

See GitHub Issues.
	
## FAQ

1. My code stalls when calling functions like `SerialPort::Read()`. This is probably because the library is set up to do a blocking read, and not enough characters have been received to allow `SerialPort::Read()` to return. Call `SerialPort::SetTimeout(0)` before the serial port is open to set a non-blocking mode.

## Changelog

See CHANGELOG.md.
