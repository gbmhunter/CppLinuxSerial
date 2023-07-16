# CppLinuxSerial

Linux serial port library written in C++.

[![CMake](https://github.com/gbmhunter/CppLinuxSerial/actions/workflows/cmake.yml/badge.svg)](https://github.com/gbmhunter/CppLinuxSerial/actions/workflows/cmake.yml)

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

	NOTE: The unit tests used to use virtual serial ports via `stty` on Linux to do more thorough testing. I ran into permission problems running stty on TravisCI after they did an update and had to remove tests (leaving almost no tests remaining). If anyone wants to add better unit tests, it is greatly welcomed! 

## Using This Project As A CMake Dependency

This project uses CMake and the export feature, so in a downstream CMake project that uses CppLinuxSerial as a dependency you should just be able to do this (thanks to https://github.com/borgmanJeremy for this contribution):

```cmake
find_package(CppLinuxSerial REQUIRED)
...
...
target_link_libraries(target CppLinuxSerial::CppLinuxSerial)
```

## Examples

```c++
#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

int main() {
	// Create serial port object and open serial port at 57600 baud, 8 data bits, no parity bit, one stop bit (8n1),
	// and no flow control
	SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(100); // Block for up to 100ms to receive data
	serialPort.Open();

	// WARNING: If using the Arduino Uno or similar, you may want to delay here, as opening the serial port causes
	// the micro to reset!

	// Write some ASCII data
	serialPort.Write("Hello");

	// Read some data back (will block for up to 100ms due to the SetTimeout(100) call above)
	std::string readData;
	serialPort.Read(readData);
	std::cout << "Read data = \"" << readData << "\"" << std::endl;

	// Close the serial port
	serialPort.Close();
}
```

If the above code was in a file called `main.cpp` and you had installed `CppLinuxSerial` following the instructions above, on a Linux system you should be able to compile the example application with:

```bash
g++ main.cpp -lCppLinuxSerial
```

If you wanted to enable flow control (hardware or software flow control), you can add it onto the end of the constructor as shown below. If you don't set them, they both default to OFF (the most common setting).

```c++
// Enabling hardware flow control
SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::ON, SoftwareFlowControl::OFF);
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

1. I get the error `Could not open device "/dev/ttyACM0". Is the device name correct and do you have read/write permissions?`, but the device is definitely there. You typically have to add your user to the `dialout` group before you can access `tty` devices.

1. My code stalls when calling functions like `SerialPort::Read()`. This is probably because the library is set up to do a blocking read, and not enough characters have been received to allow `SerialPort::Read()` to return. Call `SerialPort::SetTimeout(0)` before the serial port is open to set a non-blocking mode.

## WSL

If you want to use this library in WSL, you'll have to use usbipd to pass-through the USB device.

```
usbipd wsl list
```

```powershell
$ usbipd wsl list
BUSID  VID:PID    DEVICE                                                        STATE
1-1    046d:c332  USB Input Device                                              Not attached
1-4    13d3:5666  USB2.0 HD UVC WebCam                                          Not attached
1-5    2341:0043  Arduino Uno (COM4)                                            Not attached
1-6    046d:0a9c  Logitech G432 Gaming Headset, USB Input Device                Not attached
1-8    0b05:1837  USB Input Device                                              Not attached
1-9    8087:0a2a  Intel(R) Wireless Bluetooth(R)                                Not attached
```

Attaching the Arduino Uno (need to be done with Admin priviliges the first time around):

```
usbipd wsl attach --busid=1-5
```

`/dev/ttyACM0` now appears inside WSL, and you can use `CppLinuxSerial` with this device like usual.

NOTE: Sometimes `/dev/ttyACM0` is not part of the dialout group, so even with your user being part of that group, you will get permission denied errors when trying to access the serial port. Sometimes using `chmod` to change the permissions works:

```
sudo chmod 666 /dev/ttyACM0 
```

## Tests

Serial port testing cannot really be done easily on cloud-based CICD platforms, as serial ports and devices connected to these ports are not readily available (nor configurable). `CppLinuxSerial` relies on running tests manually on your local Linux OS, alongside a connected Arduino Uno configured to echo serial data back (at a later data this could be reconfigured to cycle through tests at different baud rates, parity settings, e.t.c).

### Prerequisites

You will need:

* Arduino Uno (or equivalent) dev kit.
* Linux OS.

Install the arduino-cli as per https://arduino.github.io/arduino-cli/0.21/installation/ on your local Linux machine.

Install the `arduino:avr` platform:

```
$ arduino-cli core install arduino:avr
```

Make sure Arduino board is detected with:

```
$ arduino-cli board list
```

### Running

Run the following bash script:

```
./test/arduino/run.sh 
```

This script will:

1. Build and install `CppLinuxSerial` onto your local Linux OS.
1. Build and upload the test Arduino firmware to the connected Arduino Uno (it assumes it's connected to `/dev/ttyACM0`).
1. Build and run the test C++ application. This sends serial data to the Uno via CppLinuxSerial and expects the data to be echoed back.

## Changelog

See CHANGELOG.md.
