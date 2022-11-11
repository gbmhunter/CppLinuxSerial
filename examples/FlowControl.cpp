#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

int main() {
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, FlowControl::NONE);
    serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();
    serialPort.Write("Hello");
    std::string readData;
	serialPort.Read(readData);
    std::cout << "readData: " << readData << std::endl;
    serialPort.Close();
    return 0;
}