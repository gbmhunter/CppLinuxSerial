///
/// \file 			SerialPort.hpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2014-01-07
/// \last-modified 	2019-05-30
/// \brief			The main serial port class.
/// \details
///					See README.rst in repo root dir for more info.

// Header guard
#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

// System headers
#include <string>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>
// #include <termios.h> // POSIX terminal control definitions (struct termios)
// #include <asm/termios.h> // Terminal control definitions (struct termios)
#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

// User headers
#include "Exception.hpp"

namespace mn {
    namespace CppLinuxSerial {

        /// \brief      Represents the baud rate "types" that can be used with the serial port. STANDARD represents all
        ///             the standard baud rates as provided by UNIX, CUSTOM represents a baud rate defined by an arbitray integer.
        enum class BaudRateType {
            STANDARD,
            CUSTOM,
        };

        /// \brief		Strongly-typed enumeration of baud rates for use with the SerialPort class
        /// \details    Specifies all the same baud rates as UNIX, as well as B_CUSTOM to specify your
        ///             own. See https://linux.die.net/man/3/cfsetispeed for list of supported UNIX baud rates.
        enum class BaudRate {
            B_0,
            B_50,
            B_75,
            B_110,
            B_134,
            B_150,
            B_200,
            B_300,
            B_600,
            B_1200,
            B_1800,
            B_2400,
            B_4800,
            B_9600,
            B_19200,
            B_38400,
            B_57600,
            B_115200,
            B_230400,
            B_460800,
            B_CUSTOM, // Placeholder 
        };

        /// \brief      Represents the state of the serial port.
        enum class State {
            CLOSED,
            OPEN,
        };

/// \brief		SerialPort object is used to perform rx/tx serial communication.
        class SerialPort {

        public:
            /// \brief		Default constructor. You must specify at least the device before calling Open().
            SerialPort();

            /// \brief		Constructor that sets up serial port with the basic (required) parameters.
            SerialPort(const std::string &device, BaudRate baudRate);

            /// \brief		Constructor that sets up serial port with the basic (required) parameters.
            SerialPort(const std::string &device, speed_t baudRate);

            /// \brief		Destructor. Closes serial port if still open.
            virtual ~SerialPort();

            /// \brief		Sets the device to use for serial port communications.
            /// \details    Method can be called when serial port is in any state.
            void SetDevice(const std::string &device);

            /// \brief      Allows the user to set a standard baud rate.
            void SetBaudRate(BaudRate baudRate);

            /// \brief      Allows the user to set a custom baud rate.
            void SetBaudRate(speed_t baudRate);

            /// \brief      Sets the read timeout (in milliseconds)/blocking mode.
            /// \details    Only call when state != OPEN. This method manupulates VMIN and VTIME.
            /// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
            ///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
            ///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
            ///             25500ms (another Linux API restriction).
            void SetTimeout(int32_t timeout_ms);

            /// \brief		Enables/disables echo.
            /// \param		value		Pass in true to enable echo, false to disable echo.
            void SetEcho(bool value);

            /// \brief		Opens the COM port for use.
            /// \throws		CppLinuxSerial::Exception if device cannot be opened.
            /// \note		Must call this before you can configure the COM port.
            void Open();

            /// \brief		Closes the COM port.
            void Close();

            /// \brief		Sends a message over the com port.
            /// \param		data		The data that will be written to the COM port.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void Write(const std::string& data);

            /// \brief		Use to read from the COM port.
            /// \param		data		The object the read characters from the COM port will be saved to.
            /// \param      wait_ms     The amount of time to wait for data. Set to 0 for non-blocking mode. Set to -1
            ///                 to wait indefinitely for new data.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void Read(std::string& data);

        private:

            /// \brief		Returns a populated termios structure for the passed in file descriptor.
            // termios GetTermios();

            /// \brief		Configures the tty device as a serial port.
            /// \warning    Device must be open (valid file descriptor) when this is called.
            void ConfigureTermios();

            // void SetTermios(termios myTermios);

            /// \brief		Returns a populated termios2 structure for the serial port pointed to by the file descriptor.
            termios2 GetTermios2();

            /// \brief      Assigns the provided tty settings to the serial port pointed to by the file descriptor.
            void SetTermios2(termios2 tty);

            /// \brief      Keeps track of the serial port's state.
            State state_;

            /// \brief      The file path to the serial port device (e.g. "/dev/ttyUSB0").
            std::string device_;

            /// \brief      The type of baud rate that the user has specified.
            BaudRateType baudRateType_;

            /// \brief      The current baud rate if baudRateType_ == STANDARD.
            BaudRate baudRateStandard_;

            /// \brief      The current baud rate if baudRateType_ == CUSTOM.
            speed_t baudRateCustom_;

            /// \brief		The file descriptor for the open file. This gets written to when Open() is called.
            int fileDesc_;

            bool echo_;

            int32_t timeout_ms_;

            std::vector<char> readBuffer_;
            unsigned char readBufferSize_B_;

            static constexpr BaudRate defaultBaudRate_ = BaudRate::B_57600;
            static constexpr int32_t defaultTimeout_ms_ = -1;
            static constexpr unsigned char defaultReadBufferSize_B_ = 255;


        };

    } // namespace CppLinuxSerial
} // namespace mn

#endif // #ifndef SERIAL_PORT_SERIAL_PORT_H
