==============================================================
serial-port-cpp
==============================================================

----------------------------------
Serial port library written in C++
----------------------------------

.. image:: https://api.travis-ci.org/gbmhunter/serial-port-cpp.png?branch=master   
	:target: https://travis-ci.org/gbmhunter/serial-port-cpp

- Author: gbmhunter <gbmhunter@gmail.com> (http://www.cladlab.com)
- Created: 2014/01/07
- Last Modified: 2014/05/15
- Version: v1.0.0.0
- Company: CladLabs
- Project: Free Code Libraries
- Language: C++
- Compiler: GCC	
- uC Model: n/a
- Computer Architecture: n/a
- Operating System: n/a
- Documentation Format: Doxygen
- License: GPLv3

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

====================== ==================== ======================================================================
Dependency             Delivery             Usage
====================== ==================== ======================================================================
<stdio.h>              Standard C library   snprintf()
====================== ==================== ======================================================================

Issues
======

See GitHub Issues.

Usage
=====

In main.c add...

::

	

	
	int main()
	{
		
	
	}
	

	
FAQ
===

1. My code stalls when calling functions like :code:`SerialPort::Read()`. This is probably because the library is set up to do a blocking read, and not enough characters have been received to allow :code:`SerialPort::Read()` to return. Use :code:`SerialPort::SetNumCharsToWait()` to determine how many characters to wait for before returning (set to 0 for non-blocking mode).


Changelog
=========

========= ========== ===================================================================================================
Version   Date       Comment
========= ========== ===================================================================================================
v1.0.0.0  2013/05/15 Initial commit. serial-port-cpp library has basic functions up and running.
========= ========== ===================================================================================================