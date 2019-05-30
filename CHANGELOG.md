# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Sphinx documentation.

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