#include <chrono>
#include <thread>

#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

// Overload to be able to print vectors to std::cout
template <typename T>
std::ostream& operator<<( std::ostream& ostrm, const std::vector<T>& vec ){
    if (vec.size() == 0) {
        return ostrm << "[]";
    }
    for( int j = 0, n = vec.size(); j < n; ++j ){
        ostrm << ",["[ !j ] << " " << vec[ j ];
    }
    return ostrm << " ]";
}

const double RX_TIMEOUT_MS = 2000;

int main() {
	// Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyACM0", BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	// SerialPort serialPort("/dev/ttyACM0", 13000);

    // Block for at most 100ms when receiving data
    // NOTE: I haven't had luck setting this to -1, Read() seems to never return
    // even though serial data has been sent to Linux
    serialPort.SetTimeout(100); 

    std::cout << "Opening /dev/ttyACM0 at 9600 baud, 8n1..." << std::flush;
	serialPort.Open();
    std::cout << "OK." << std::endl;

    std::cout << "Sleeping to allow Arduino to restart (happens after opening serial port)..." << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "OK." << std::endl;

	// Write some ASCII data and read it back
    std::cout << "Writing string data..." << std::flush;
    std::string txDataString = "Hello";
	serialPort.Write(txDataString);
    std:: cout << "OK." << std::endl;
	
    std::cout << "Reading string data..." << std::flush;
    std::string rxDataString;
    auto t_start = std::chrono::high_resolution_clock::now();
    while(1) {
        serialPort.Read(rxDataString);
        if (rxDataString == txDataString) {
            std::cout << "OK." << std::endl;
            break;
        }
        auto t_now = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
        if (elapsed_time_ms >= RX_TIMEOUT_MS) {
            std::cout << "ERROR: Did not receive the string data \"Hello\" from Arduino." << std::endl;
            return -1;
        }
    }

    // Write some binary data and read it back
    std::cout << "Writing binary data..." << std::flush;
    std::vector<uint8_t> txDataBinary{ 1, 2, 3, 4, 5 };
	serialPort.WriteBinary(txDataBinary);
    std:: cout << "OK." << std::endl;
	
    std::cout << "Reading binary data..." << std::flush;
    std::vector<uint8_t> rxDataBinary;
    t_start = std::chrono::high_resolution_clock::now();
    while(1) {
        serialPort.ReadBinary(rxDataBinary);
        if (rxDataBinary == txDataBinary) {
            std::cout << "OK." << std::endl;
            break;
        }
        auto t_now = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_now - t_start).count();
        if (elapsed_time_ms >= RX_TIMEOUT_MS) {
            std::cout << "ERROR: Did not receive the binary data from Arduino." << std::endl;
            return -1;
        }
    }

	// Close the serial port
	serialPort.Close();
    return 0;
}