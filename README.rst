==============
CppLinuxSerial
==============

----------------------------------
Serial port library written in C++
----------------------------------

.. image:: https://travis-ci.org/mbedded-ninja/CppLinuxSerial.svg?branch=master
	:target: https://travis-ci.org/mbedded-ninja/CppLinuxSerial

.. role:: bash(code)
	:language: bash

Description
===========

Library for communicating with COM ports on a Linux system.

Uses fstream to the file I/O.

Installation
============

1. Clone the git repo onto your local storage.

2. Run `make all` to compile and run unit tests. Do not worry about error messages being printed when unit tests are run, the unit tests are designed to specifically cause errors to test the response.

3. To include serial-port-cpp into your embedded (or otherwise) firmware/software project, copy the repo into your project folder (or other suitable place), include the file "/api/SerialPortApi.hpp" from your C++ code, and make sure to compile all the files within "/src/".


Dependencies
============

The following table lists all of the libraries dependencies.

====================== ======================================================================
Dependency             Comments
====================== ======================================================================
C++14                  C++14 used for strongly typed enums, std::chrono and literals.
<stdio.h>              snprintf()
stty                   Used in unit tests to verify the serial port is configured correctly.
====================== ======================================================================

Issues
======

See GitHub Issues.

Usage
=====

Nothing here yet...
	
FAQ
===

1. My code stalls when calling functions like :code:`SerialPort::Read()`. This is probably because the library is set up to do a blocking read, and not enough characters have been received to allow :code:`SerialPort::Read()` to return. Use :code:`SerialPort::SetNumCharsToWait()` to determine how many characters to wait for before returning (set to 0 for non-blocking mode).


Changelog
=========

See CHANGELOG.md.