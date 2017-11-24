///
/// \file 			SerialPort.hpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> ()
/// \created		2014-01-07
/// \last-modified 	2017-11-23
/// \brief			The main serial port class.
/// \details
///					See README.rst in repo root dir for more info.

// Header guard
#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

// System headers
#include <fstream>		// For file I/O (reading/writing to COM port)
#include <sstream>
#include <termios.h> 	// POSIX terminal control definitions (struct termios)

// User headers

namespace mn {
namespace SerialPort
{

	/// \brief		Strongly-typed enumeration of baud rates for use with the SerialPort class
	enum class BaudRates {
		none,
		b9600,
		b57600
	};

	//! @brief		SerialPort object is used to perform rx/tx serial communication.
	class SerialPort
	{

	public:

		//! @brief		Constructor
		SerialPort();

		//! @brief		Destructor
		virtual ~SerialPort();

		//! @brief		Sets the file path to use for communications. The file path must be set before Open() is called, otherwise Open() will return an error.
		void SetFilePath(std::string filePath);

		void SetBaudRate(BaudRates baudRate);

		//! @brief		Controls what happens when Read() is called.
		//! @param		numOfCharToWait		Minimum number of characters to wait for before returning. Set to 0 for non-blocking mode.
		void SetNumCharsToWait(uint32_t numCharsToWait);

		//! @brief		Enables/disables echo.
		//! param		echoOn		Pass in true to enable echo, false to disable echo.
		void EnableEcho(bool echoOn);

		//! @brief		Opens the COM port for use.
		//! @throws		{std::runtime_error} if filename has not been set.
		//!				{std::system_error} if system open() operation fails.
		//! @note		Must call this before you can configure the COM port.
		void Open();

		//! @brief		Sets all settings for the com port to common defaults.
		void SetEverythingToCommonDefaults();

		//! @brief		Closes the COM port.
		void Close();

		//! @brief		Sends a message over the com port.
		//! @param		str		Reference to an string containing the characters to write to the COM port.
		//! @throws		{std::runtime_error} if filename has not been set.
		//!				{std::system_error} if system write() operation fails.
		void Write(std::string* str);

		//! @brief		Use to read from the COM port.
		//! @param		str		Reference to a string that the read characters from the COM port will be saved to.
		//! @throws		{std::runtime_error} if filename has not been set.
		//!				{std::system_error} if system read() operation fails.
		void Read(std::string* str);

	private:

		std::string filePath;

		BaudRates baudRate;

		//! @brief		The file descriptor for the open file. This gets written to when Open() is called.
		int fileDesc;

		//! @brief		Returns a populated termios structure for the passed in file descriptor.
		termios GetTermios();

		void SetTermios(termios myTermios);

	};

} // namespace SerialPort
} // namespace mn

#endif // #ifndef SERIAL_PORT_SERIAL_PORT_H
