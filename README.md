# CppLinuxSerial

Linux serial port library written in C++.

[![Build Status](https://travis-ci.org/gbmhunter/CppLinuxSerial.svg?branch=master)](https://travis-ci.org/gbmhunter/CppLinuxSerial)

## Description

Library for communicating with COM ports on a Linux system.

* Simple API
* Supports custom baud rates
* cmake based build system

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

1. To run the unit tests (NOTE: because this uses virtual serial ports via `stty`, this only works on Linux!):

	```bash
	$ make run_unit_tests
	```

	If you get errors such as `Could not open device /dev/ttyS10. Is the device name correct and do you have read/write permission?" thrown in the test fixture's constructor.`, it is probably an issue with either creating the virtual serial ports or permissions to access them.

## Examples

```c++
#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

int main() {
	// Create serial port object and open serial port
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();

	// Write some ASCII datae
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

For more examples, see the files in `test/`.

## Issues

See GitHub Issues.
	
## FAQ

1. My code stalls when calling functions like `SerialPort::Read()`. This is probably because the library is set up to do a blocking read, and not enough characters have been received to allow `SerialPort::Read()` to return. Call `SerialPort::SetTimeout(0)` before the serial port is open to set a non-blocking mode.

## Changelog

See CHANGELOG.md.