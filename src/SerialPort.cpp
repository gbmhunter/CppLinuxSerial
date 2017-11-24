//!
//! @file 			SerialPort.cpp
//! @author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
//! @created		2014-01-07
//! @last-modified 	2017-11-23
//! @brief			The main serial port class.
//! @details
//!					See README.rst in repo root dir for more info.

#include <iostream>
#include <sstream>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions (struct termios)
#include <system_error>	// For throwing std::system_error

#include "CppLinuxSerial/SerialPort.hpp"

namespace mn {
namespace CppLinuxSerial {

	SerialPort::SerialPort() {
	}

	SerialPort::SerialPort(const std::string& device, BaudRate baudRate) {
		device_ = device;
		baudRate_ = baudRate;
	}

	SerialPort::~SerialPort() {
        try {
            Close();
        } catch(...) {
            // We can't do anything about this!
        }
	}

	void SerialPort::SetDevice(const std::string& device)
	{		
		device_ = device;
        ConfigureTermios();
	}

	void SerialPort::SetBaudRate(BaudRate baudRate)
	{
		baudRate_ = baudRate;
        ConfigureTermios();
	}

	void SerialPort::Open()
	{

		std::cout << "Attempting to open COM port \"" << device_ << "\"." << std::endl;

		if(device_.size() == 0) {
			//this->sp->PrintError(SmartPrint::Ss() << "Attempted to open file when file path has not been assigned to.");
			//return false;

			throw std::runtime_error("Attempted to open file when file path has not been assigned to.");
		}

		// Attempt to open file
		//this->fileDesc = open(this->filePath, O_RDWR | O_NOCTTY | O_NDELAY);

		// O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both read/write access
		// 3rd, optional parameter is mode_t mode
		fileDesc_ = open(device_.c_str(), O_RDWR);

		// Check status
		if(fileDesc_ == -1) {
			// Could not open COM port
		    //this->sp->PrintError(SmartPrint::Ss() << "Unable to open " << this->filePath << " - " << strerror(errno));
		    //return false;

		    throw std::runtime_error("Could not open device " + device_ + ". Is the device name correct and do you have read/write permission?");
		}

        ConfigureTermios();

		std::cout << "COM port opened successfully." << std::endl;

		// If code reaches here, open and config must of been successful

	}

	void SerialPort::EnableEcho(bool echoOn)
	{
		termios settings = this->GetTermios();
		settings.c_lflag = echoOn
					   ? (settings.c_lflag |   ECHO )
					   : (settings.c_lflag & ~(ECHO));
		//tcsetattr( STDIN_FILENO, TCSANOW, &settings );
		this->SetTermios(settings);
	}

	void SerialPort::ConfigureTermios()
	{
		std::cout << "Configuring COM port \"" << device_ << "\"." << std::endl;

		//================== CONFIGURE ==================//

		termios tty = GetTermios();

		//================= (.c_cflag) ===============//

		tty.c_cflag     &=  ~PARENB;       	// No parity bit is added to the output characters
		tty.c_cflag     &=  ~CSTOPB;		// Only one stop-bit is used
		tty.c_cflag     &=  ~CSIZE;			// CSIZE is a mask for the number of bits per character
		tty.c_cflag     |=  CS8;			// Set to 8 bits per character
		tty.c_cflag     &=  ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
		tty.c_cflag     |=  CREAD | CLOCAL;     				// Turn on READ & ignore ctrl lines (CLOCAL = 1)


        //===================== BAUD RATE =================//

        switch(baudRate_) {
            case BaudRate::B_9600:
                cfsetispeed(&tty, B9600);
                cfsetospeed(&tty, B9600);
                break;
            case BaudRate::B_38400:
                cfsetispeed(&tty, B38400);
                cfsetospeed(&tty, B38400);
                break;
            case BaudRate::B_57600:
                cfsetispeed(&tty, B57600);
                cfsetospeed(&tty, B57600);
                break;
            case BaudRate::B_115200:
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);
                break;
            case BaudRate::CUSTOM:
                // See https://gist.github.com/kennethryerson/f7d1abcf2633b7c03cf0
                throw std::runtime_error("Custom baud rate not yet supported.");
            default:
                throw std::runtime_error(std::string() + "baudRate passed to " + __PRETTY_FUNCTION__ + " unrecognized.");
        }

		//===================== (.c_oflag) =================//

		tty.c_oflag     =   0;              // No remapping, no delays
		tty.c_oflag     &=  ~OPOST;			// Make raw

		//================= CONTROL CHARACTERS (.c_cc[]) ==================//

		// c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
		// Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode
		//tty.c_cc[VMIN]      =   1;
		SetNumCharsToWait(1);

		// c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
		// Only meaningful when port is set to non-canonical mode
		tty.c_cc[VTIME]     =   5;          // 0.5 seconds read timeout


		//======================== (.c_iflag) ====================//

		tty.c_iflag     &= ~(IXON | IXOFF | IXANY);			// Turn off s/w flow ctrl
		tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

		//=========================== LOCAL MODES (c_lflag) =======================//

		// Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
		// read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
		tty.c_lflag		&= ~ICANON;								// Turn off canonical input, which is suitable for pass-through
		tty.c_lflag		&= ~ECHO;								// Turn off echo
		tty.c_lflag		&= ~ECHOE;								// Turn off echo erase (echo erase only relevant if canonical input is active)
		tty.c_lflag		&= ~ECHONL;								//
		tty.c_lflag		&= ~ISIG;								// Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

		// Try and use raw function call
		//cfmakeraw(&tty);

		this->SetTermios(tty);

		/*
		// Flush port, then apply attributes
		tcflush(this->fileDesc, TCIFLUSH);

		if(tcsetattr(this->fileDesc, TCSANOW, &tty) != 0)
		{
			// Error occurred
			this->sp->PrintError(SmartPrint::Ss() << "Could not apply terminal attributes for \"" << this->filePath << "\" - " << strerror(errno));
			return;
		}*/
	}

	void SerialPort::SetNumCharsToWait(uint32_t numCharsToWait) {
		// Get current termios struct
		termios myTermios = GetTermios();

		// Save the number of characters to wait for
		// to the control register
		myTermios.c_cc[VMIN] = numCharsToWait;

		// Save termios back
		SetTermios(myTermios);
	}

	void SerialPort::Write(const std::string& data) {
		if(fileDesc_ == 0) {
			//this->sp->PrintError(SmartPrint::Ss() << );
			//return false;

			throw std::runtime_error("SendMsg called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.c_str(), data.size());

		// Check status
		if (writeResult == -1) {
			// Could not open COM port
			//this->sp->PrintError(SmartPrint::Ss() << "Unable to write to \"" << this->filePath << "\" - " << strerror(errno));
			//return false;

			throw std::system_error(EFAULT, std::system_category());
		}

		// If code reaches here than write must of been successful
	}

	void SerialPort::Read(std::string& data)
	{
		if(fileDesc_ == 0) {
			//this->sp->PrintError(SmartPrint::Ss() << "Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
			//return false;
			throw std::runtime_error("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		// Allocate memory for read buffer
		char buf [256];
		memset (&buf, '\0', sizeof buf);

		// Read from file
		int n = read(fileDesc_, &buf, sizeof(buf));

		// Error Handling
		if(n < 0) {
			// Read was unsuccessful
			throw std::system_error(EFAULT, std::system_category());
		}

		if(n > 0) {
			//this->sp->PrintDebug(SmartPrint::Ss() << "\"" << n << "\" characters have been read from \"" << this->filePath << "\"");
			// Characters have been read
			buf[n] = '\0';
			//printf("%s\r\n", buf);
			data.append(buf);
			//std::cout << *str << " and size of string =" << str->size() << "\r\n";
		}

		// If code reaches here, read must of been successful
	}

	termios SerialPort::GetTermios()
	{
        if(fileDesc_ == -1)
            throw std::runtime_error("GetTermios() called but file descriptor was not valid.");

		struct termios tty;
		memset(&tty, 0, sizeof(tty));

		// Get current settings (will be stored in termios structure)
		if(tcgetattr(fileDesc_, &tty) != 0)
		{
			// Error occurred
			std::cout << "Could not get terminal attributes for \"" << device_ << "\" - " << strerror(errno) << std::endl;
			throw std::system_error(EFAULT, std::system_category());
			//return false;
		}

		return tty;
	}

	void SerialPort::SetTermios(termios myTermios)
	{
		// Flush port, then apply attributes
		tcflush(this->fileDesc_, TCIFLUSH);

		if(tcsetattr(this->fileDesc_, TCSANOW, &myTermios) != 0)
		{
			// Error occurred
			std::cout << "Could not apply terminal attributes for \"" << device_ << "\" - " << strerror(errno) << std::endl;
			throw std::system_error(EFAULT, std::system_category());

		}

		// Successful!
	}

    void SerialPort::Close() {
        if(fileDesc_ != -1) {
            auto retVal = close(fileDesc_);
            if(retVal != 0)
                throw std::runtime_error("Tried to close serial port " + device_ + ", but close() failed.");

            fileDesc_ = -1;
        }

        state_ = State::CLOSED;
    }

} // namespace CppLinuxSerial
} // namespace mn
