//!
//! @file 			SerialPort.cpp
//! @author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
//! @created		2014-01-07
//! @last-modified 	2017-11-27
//! @brief			The main serial port class.
//! @details
//!					See README.rst in repo root dir for more info.

// System includes
#include <iostream>
#include <sstream>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
// #include <termios.h> 	// POSIX terminal control definitions (struct termios)
#include <system_error>	// For throwing std::system_error
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
// #include <asm/termios.h> // Terminal control definitions (struct termios)
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

// User includes
#include "CppLinuxSerial/Exception.hpp"
#include "CppLinuxSerial/SerialPort.hpp"

#define    BOTHER 0010000

namespace mn {
namespace CppLinuxSerial {

	SerialPort::SerialPort() {
        echo_ = false;
        timeout_ms_ = defaultTimeout_ms_;
		baudRateType_ = BaudRateType::STANDARD;
        baudRateStandard_ = defaultBaudRate_;
        readBufferSize_B_ = defaultReadBufferSize_B_;
        readBuffer_.reserve(readBufferSize_B_);
		state_ = State::CLOSED;
	}

	SerialPort::SerialPort(const std::string& device, BaudRate baudRate) :
            SerialPort() {
		device_ = device;
		baudRateType_ = BaudRateType::STANDARD;
        baudRateStandard_ = baudRate;
	}

	SerialPort::SerialPort(const std::string& device, speed_t baudRate) :
            SerialPort() {
		device_ = device;
		baudRateType_ = BaudRateType::CUSTOM;
        baudRateCustom_ = baudRate;
	}

	SerialPort::SerialPort(const std::string& device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits) :
            SerialPort() {
		device_ = device;
		baudRateType_ = BaudRateType::STANDARD;
        baudRateStandard_ = baudRate;
		numDataBits_ = numDataBits;
		parity_ = parity;
		numStopBits_ = numStopBits;
	}

	SerialPort::~SerialPort() {
        try {
            Close();
        } catch(...) {
            // We can't do anything about this!
            // But we don't want to throw within destructor, so swallow
        }
	}

	void SerialPort::SetDevice(const std::string& device) {
		device_ = device;
        if(state_ == State::OPEN)
        	ConfigureTermios();
	}

	void SerialPort::SetBaudRate(BaudRate baudRate)	{		
		baudRateType_ = BaudRateType::STANDARD;
		baudRateStandard_ = baudRate;
        if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetBaudRate(speed_t baudRate)	{		
		baudRateType_ = BaudRateType::CUSTOM;
		baudRateCustom_ = baudRate;
        if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetNumDataBits(NumDataBits numDataBits) {
		numDataBits_ = numDataBits;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetParity(Parity parity) {
		parity_ = parity;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::SetNumStopBits(NumStopBits numStopBits) {
		numStopBits_ = numStopBits;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

	void SerialPort::Open()
	{

		// std::cout << "Attempting to open COM port \"" << device_ << "\"." << std::endl;

		if(device_.empty()) {
			THROW_EXCEPT("Attempted to open file when file path has not been assigned to.");
		}

		// Attempt to open file
		//this->fileDesc = open(this->filePath, O_RDWR | O_NOCTTY | O_NDELAY);

		// O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both read/write access
		// 3rd, optional parameter is mode_t mode
		fileDesc_ = open(device_.c_str(), O_RDWR);

		// Check status
		if(fileDesc_ == -1) {
		    THROW_EXCEPT("Could not open device " + device_ + ". Is the device name correct and do you have read/write permission?");
		}

        ConfigureTermios();

		// std::cout << "COM port opened successfully." << std::endl;
        state_ = State::OPEN;
	}

	void SerialPort::SetEcho(bool value) {
        echo_ = value;
        ConfigureTermios();
	}

	void SerialPort::ConfigureTermios()
	{
		// std::cout << "Configuring COM port \"" << device_ << "\"." << std::endl;

		//================== CONFIGURE ==================//

		// termios tty = GetTermios();
		termios2 tty = GetTermios2();

		//================= (.c_cflag) ===============//

		// Set num. data bits
		// See https://man7.org/linux/man-pages/man3/tcflush.3.html
		tty.c_cflag     &=  ~CSIZE;			// CSIZE is a mask for the number of bits per character
		switch(numDataBits_) {
			case NumDataBits::FIVE:
				tty.c_cflag     |=  CS5;
				break;
			case NumDataBits::SIX:
				tty.c_cflag     |=  CS6;
				break;
			case NumDataBits::SEVEN:
				tty.c_cflag     |=  CS7;
				break;
			case NumDataBits::EIGHT:
				tty.c_cflag     |=  CS8;
				break;
			default:
				THROW_EXCEPT("numDataBits_ value not supported!");
		}
		
		// Set parity
		// See https://man7.org/linux/man-pages/man3/tcflush.3.html
		switch(parity_) {
			case Parity::NONE:
				tty.c_cflag     &=  ~PARENB;
				break;
			case Parity::EVEN:	
				tty.c_cflag 	|=   PARENB;
				tty.c_cflag		&=	 ~PARODD; // Clearing PARODD makes the parity even
				break;
			case Parity::ODD:
				tty.c_cflag     |=   PARENB;
				tty.c_cflag		|=	 PARODD;
				break;
			default:
				THROW_EXCEPT("parity_ value not supported!");

		}

		// Set num. stop bits
		switch(numStopBits_) {
			case NumStopBits::ONE:
				tty.c_cflag     &=  ~CSTOPB;
				break;
			case NumStopBits::TWO:
				tty.c_cflag     |=  CSTOPB;
				break;
			default:
				THROW_EXCEPT("numStopBits_ value not supported!");
		}

		tty.c_cflag     &=  ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
		tty.c_cflag     |=  CREAD | CLOCAL;     				// Turn on READ & ignore ctrl lines (CLOCAL = 1)


        //===================== BAUD RATE =================//

		// We used to use cfsetispeed() and cfsetospeed() with the B... macros, but this didn't allow
		// us to set custom baud rates. So now to support both standard and custom baud rates lets
		// just make everything "custom". This giant switch statement could be replaced with a map/lookup
		// in the future
		if (baudRateType_ == BaudRateType::STANDARD) {
			tty.c_cflag &= ~CBAUD;
			tty.c_cflag |= CBAUDEX;
			switch(baudRateStandard_) {
				case BaudRate::B_0:
					// cfsetispeed(&tty, B0);
					// cfsetospeed(&tty, B0);
					tty.c_ispeed = 0;
					tty.c_ospeed = 0;
					break;
				case BaudRate::B_50:
					// cfsetispeed(&tty, B50);
					// cfsetospeed(&tty, B50);
					tty.c_ispeed = 50;
					tty.c_ospeed = 50;
					break;
				case BaudRate::B_75:
				    // cfsetispeed(&tty, B75);
				    // cfsetospeed(&tty, B75);
					tty.c_ispeed = 75;
					tty.c_ospeed = 75;
				    break;
				case BaudRate::B_110:
				    // cfsetispeed(&tty, B110);
				    // cfsetospeed(&tty, B110);
					tty.c_ispeed = 110;
					tty.c_ospeed = 110;
				    break;
				case BaudRate::B_134:
				    // cfsetispeed(&tty, B134);
				    // cfsetospeed(&tty, B134);
					tty.c_ispeed = 134;
					tty.c_ospeed = 134;
				    break;
				case BaudRate::B_150:
				    // cfsetispeed(&tty, B150);
				    // cfsetospeed(&tty, B150);
					tty.c_ispeed = 150;
					tty.c_ospeed = 150;
				    break;
				case BaudRate::B_200:
				    // cfsetispeed(&tty, B200);
				    // cfsetospeed(&tty, B200);
					tty.c_ispeed = 200;
					tty.c_ospeed = 200;
				    break;
				case BaudRate::B_300:
				    // cfsetispeed(&tty, B300);
				    // cfsetospeed(&tty, B300);
					tty.c_ispeed = 300;
					tty.c_ospeed = 300;
				    break;
				case BaudRate::B_600:
				    // cfsetispeed(&tty, B600);
				    // cfsetospeed(&tty, B600);
					tty.c_ispeed = 600;
					tty.c_ospeed = 600;
				    break;
				case BaudRate::B_1200:
				    // cfsetispeed(&tty, B1200);
				    // cfsetospeed(&tty, B1200);
					tty.c_ispeed = 1200;
					tty.c_ospeed = 1200;
				    break;
				case BaudRate::B_1800:
				    // cfsetispeed(&tty, B1800);
				    // cfsetospeed(&tty, B1800);
					tty.c_ispeed = 1800;
					tty.c_ospeed = 1800;
				    break;
				case BaudRate::B_2400:
				    // cfsetispeed(&tty, B2400);
				    // cfsetospeed(&tty, B2400);
					tty.c_ispeed = 2400;
					tty.c_ospeed = 2400;
				    break;
				case BaudRate::B_4800:
				    // cfsetispeed(&tty, B4800);
				    // cfsetospeed(&tty, B4800);
					tty.c_ispeed = 4800;
					tty.c_ospeed = 4800;
				    break;
				case BaudRate::B_9600:
					// cfsetispeed(&tty, B9600);
					// cfsetospeed(&tty, B9600);
					tty.c_ispeed = 9600;
					tty.c_ospeed = 9600;
					break;
				case BaudRate::B_19200:
				    // cfsetispeed(&tty, B19200);
				    // cfsetospeed(&tty, B19200);
					tty.c_ispeed = 19200;
					tty.c_ospeed = 19200;
				    break;
				case BaudRate::B_38400:
				    // cfsetispeed(&tty, B38400);
				    // cfsetospeed(&tty, B38400);
					tty.c_ispeed = 38400;
					tty.c_ospeed = 38400;
				    break;
				case BaudRate::B_57600:
				    // cfsetispeed(&tty, B57600);
				    // cfsetospeed(&tty, B57600);
					tty.c_ispeed = 57600;
					tty.c_ospeed = 57600;
				    break;
				case BaudRate::B_115200:
				    // cfsetispeed(&tty, B115200);
				    // cfsetospeed(&tty, B115200);
					tty.c_ispeed = 115200;
					tty.c_ospeed = 115200;
				    break;
				case BaudRate::B_230400:
				    // cfsetispeed(&tty, B230400);
				    // cfsetospeed(&tty, B230400);
					tty.c_ispeed = 230400;
					tty.c_ospeed = 230400;
				    break;
				case BaudRate::B_460800:
				    // cfsetispeed(&tty, B460800);
				    // cfsetospeed(&tty, B460800);
					tty.c_ispeed = 460800;
					tty.c_ospeed = 460800;
				    break;
				default:
					throw std::runtime_error(std::string() + "baudRate passed to " + __PRETTY_FUNCTION__ + " unrecognized.");
			}
		}
		// This does no different than STANDARD atm, but let's keep
		// them separate for now....
		else if (baudRateType_ == BaudRateType::CUSTOM)
		{
			tty.c_cflag &= ~CBAUD;
			tty.c_cflag |= CBAUDEX;
			// tty.c_cflag |= BOTHER;
			tty.c_ispeed = baudRateCustom_;
			tty.c_ospeed = baudRateCustom_;


			// #include <linux/serial.h>
			// // configure port to use custom speed instead of 38400
			// struct serial_struct ss;
			// ioctl(fileDesc_, TIOCGSERIAL, &ss);
			// ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
			// ss.custom_divisor = (ss.baud_base + (baudRateCustom_ / 2)) / baudRateCustom_;
			// int closestSpeed = ss.baud_base / ss.custom_divisor;

			// if (closestSpeed < baudRateCustom_ * 98 / 100 || closestSpeed > baudRateCustom_ * 102 / 100) {
			// 	printf("Cannot set serial port speed to %d. Closest possible is %d\n", baudRateCustom_, closestSpeed);
			// }

			// ioctl(fileDesc_, TIOCSSERIAL, &ss);

			// cfsetispeed(&tty, B38400);
			// cfsetospeed(&tty, B38400);
		}
		else
		{
			// Should never get here, bug in this libraries code!
			assert(false);
		}

		//===================== (.c_oflag) =================//

		tty.c_oflag     =   0;              // No remapping, no delays
		tty.c_oflag     &=  ~OPOST;			// Make raw

		//================= CONTROL CHARACTERS (.c_cc[]) ==================//

		// c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
		// Only meaningful when port is set to non-canonical mode
        // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
        // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
        // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
        // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
        //                      after first character has elapsed
        // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
        // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

        if(timeout_ms_ == -1) {
            // Always wait for at least one byte, this could
            // block indefinitely
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 1;
        } else if(timeout_ms_ == 0) {
            // Setting both to 0 will give a non-blocking read
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 0;
        } else if(timeout_ms_ > 0) {
            tty.c_cc[VTIME] = (cc_t)(timeout_ms_/100);    // 0.5 seconds read timeout
            tty.c_cc[VMIN] = 0;
        }

		//======================== (.c_iflag) ====================//

		tty.c_iflag     &= ~(IXON | IXOFF | IXANY);			// Turn off s/w flow ctrl
		tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

		//=========================== LOCAL MODES (c_lflag) =======================//

		// Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
		// read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
		tty.c_lflag		&= ~ICANON;								// Turn off canonical input, which is suitable for pass-through
		// Configure echo depending on echo_ boolean
        if(echo_) {
			tty.c_lflag |= ECHO;
		} else {
			tty.c_lflag &= ~(ECHO);
		}
		tty.c_lflag		&= ~ECHOE;								// Turn off echo erase (echo erase only relevant if canonical input is active)
		tty.c_lflag		&= ~ECHONL;								//
		tty.c_lflag		&= ~ISIG;								// Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters


		// Try and use raw function call
		//cfmakeraw(&tty);

		// this->SetTermios(tty);
		this->SetTermios2(tty);

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

	void SerialPort::Write(const std::string& data) {

        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.c_str(), data.size());

		// Check status
		if (writeResult == -1) {
			throw std::system_error(EFAULT, std::system_category());
		}
	}

       void SerialPort::WriteBinary(const std::vector<uint8_t>& data) {

        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.data(), data.size());

		// Check status
		if (writeResult == -1) {
			throw std::system_error(EFAULT, std::system_category());
		}
	}

	void SerialPort::Read(std::string& data)
	{
        data.clear();

		if(fileDesc_ == 0) {
			//this->sp->PrintError(SmartPrint::Ss() << "Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
			//return false;
			THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		// Allocate memory for read buffer
//		char buf [256];
//		memset (&buf, '\0', sizeof buf);

		// Read from file
        // We provide the underlying raw array from the readBuffer_ vector to this C api.
        // This will work because we do not delete/resize the vector while this method
        // is called
		ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

		// Error Handling
		if(n < 0) {
			// Read was unsuccessful
			throw std::system_error(EFAULT, std::system_category());
		}

		if(n > 0) {

//			buf[n] = '\0';
			//printf("%s\r\n", buf);
//			data.append(buf);
            data = std::string(&readBuffer_[0], n);
			//std::cout << *str << " and size of string =" << str->size() << "\r\n";
		}

		// If code reaches here, read must of been successful
	}

	void SerialPort::ReadBinary(std::vector<uint8_t>& data)
	{
        data.clear();

		if(fileDesc_ == 0) {
			//this->sp->PrintError(SmartPrint::Ss() << "Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
			//return false;
			THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		// Read from file
        // We provide the underlying raw array from the readBuffer_ vector to this C api.
        // This will work because we do not delete/resize the vector while this method
        // is called
		ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

		// Error Handling
		if(n < 0) {
			// Read was unsuccessful
			throw std::system_error(EFAULT, std::system_category());
		}

		if(n > 0) {
            copy(readBuffer_.begin(), readBuffer_.begin() + n, back_inserter(data));
		}

		// If code reaches here, read must of been successful
	}

	// termios SerialPort::GetTermios() {
    //     if(fileDesc_ == -1)
    //         throw std::runtime_error("GetTermios() called but file descriptor was not valid.");

	// 	struct termios tty;
	// 	memset(&tty, 0, sizeof(tty));

	// 	// Get current settings (will be stored in termios structure)
	// 	if(tcgetattr(fileDesc_, &tty) != 0)
	// 	{
	// 		// Error occurred
	// 		std::cout << "Could not get terminal attributes for \"" << device_ << "\" - " << strerror(errno) << std::endl;
	// 		throw std::system_error(EFAULT, std::system_category());
	// 		//return false;
	// 	}

	// 	return tty;
	// }

	// void SerialPort::SetTermios(termios myTermios)
	// {
	// 	// Flush port, then apply attributes
	// 	tcflush(fileDesc_, TCIFLUSH);

	// 	if(tcsetattr(fileDesc_, TCSANOW, &myTermios) != 0)
	// 	{
	// 		// Error occurred
	// 		std::cout << "Could not apply terminal attributes for \"" << device_ << "\" - " << strerror(errno) << std::endl;
	// 		throw std::system_error(EFAULT, std::system_category());

	// 	}

	// 	// Successful!
	// }

	termios2 SerialPort::GetTermios2()
	{
		struct termios2 term2;

        ioctl(fileDesc_, TCGETS2, &term2);

		return term2;

        // term2.c_cflag &= ~CBAUD;  /* Remove current BAUD rate */
        // term2.c_cflag |= BOTHER;  /* Allow custom BAUD rate using int input */
        // term2.c_ispeed = speed;   /* Set the input BAUD rate */
        // term2.c_ospeed = speed;   /* Set the output BAUD rate */

        // ioctl(fd, TCSETS2, &term2);
	}

	void SerialPort::SetTermios2(termios2 tty)
	{
		ioctl(fileDesc_, TCSETS2, &tty);
	}

    void SerialPort::Close() {
        if(fileDesc_ != -1) {
            auto retVal = close(fileDesc_);
            if(retVal != 0)
                THROW_EXCEPT("Tried to close serial port " + device_ + ", but close() failed.");

            fileDesc_ = -1;
        }

        state_ = State::CLOSED;
    }

    void SerialPort::SetTimeout(int32_t timeout_ms) {
        if(timeout_ms < -1)
            THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was < -1, which is invalid.");
        if(timeout_ms > 25500)
            THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was > 25500, which is invalid.");
        if(state_ == State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called while state == OPEN.");
        timeout_ms_ = timeout_ms;
    }
    
    int32_t SerialPort::Available() {
		if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");
        int32_t ret = 0;
        ioctl(fileDesc_, FIONREAD, &ret);
        return ret;
        
    }

} // namespace CppLinuxSerial
} // namespace mn
