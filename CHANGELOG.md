# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [v2.1.2] - 2021-07-01

- Removed download of gtest if tests are not built.
- Install headers correctly when used as a cmake submodule (FetchContent).

Big thanks to https://github.com/MadsAndreasen-Zoetis for these improvements.

## [v2.1.1] - 2021-04-05

- Fixed bug where `echo` was not being set correctly.

## [v2.1.0] - 2020-11-08

### Added
- Support for custom baud rates.
- Support for all standard UNIX baud rates.
- Improved Doxygen documentation.
- Improved README.md documentation.

### Removed
- Dependencies from the README, they weren't that useful and were not accurate anyway.

## [v2.0.3] - 2020-10-13

### Added
- Added example of how to use the library to the README.

### Fixed
- Fixed CMake not using C++14 by specifying `set(CMAKE_CXX_STANDARD 14)` rather than just `add_definitions(-std=c++14)`.

### Removed
- Removed all unit tests that were using virtual serial ports for testing, as these are broken due to a changing TravisCI OS environment. This needs fixing at a later date.

## [v2.0.2] - 2020-07-07

### Added
- Sphinx documentation.
- Unit tests for testing `SetTimeout()`.

### Fixed
- Serial port state is set to `CLOSED` on initialization.
- Fixed build commands in README.

## [v2.0.1] - 2017-11-27

### Fixed
- Fixed link to TravisCI image in README.

## [v2.0.0] - 2017-11-27

### Added
- Added CMake build support.
- Added basic, config and read/write unit tests using gtest.
- Improved read() performance due to removal of buffer creation on every call.
- TravisCI configuration file.
- Build script under `tools/`.

### Changed
- Updated serial port to use C++14.
- Changed library name from serial-port to CppLinuxSerial.
- Updated Doxygen comments.

## [v1.0.1] - 2014-05-21
 
### Changed
- Added ability to enable/disable echo with 'SerialPort::EnableEcho()'.

## [v1.0.0] - 2014-05-15

### Added
- Initial commit. serial-port-cpp library has basic functions up and running.

[Unreleased]: https://github.com/mbedded-ninja/CppLinuxSerial/compare/v2.0.1...HEAD
[v2.0.1]: https://github.com/mbedded-ninja/CppLinuxSerial/compare/v2.0.1...v2.0.0
[v2.0.0]: https://github.com/mbedded-ninja/CppLinuxSerial/compare/v2.0.0...v1.0.1
[v1.0.1]: https://github.com/mbedded-ninja/CppLinuxSerial/compare/v1.0.1...v1.0.0