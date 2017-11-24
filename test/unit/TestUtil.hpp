///
/// \file 			TestUtil.hpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2017-11-24
/// \last-modified 	2017-11-24
/// \brief			Contains utility methods to help with testing.
/// \details
///					See README.rst in repo root dir for more info.

#ifndef MN_CPP_LINUX_SERIAL_TEST_UTIL_H_
#define MN_CPP_LINUX_SERIAL_TEST_UTIL_H_

// System includes
#include <string>
#include <array>
#include <memory>
#include <iostream>
#include <thread>
#include <chrono>

// 3rd party includes


using namespace std::literals;


namespace mn {
    namespace CppLinuxSerial {

        class TestUtil {

        public:
            /// \brief      Executes a command on the Linux command-line.
            /// \details    Blocks until command is complete.
            /// \throws     std::runtime_error is popen() fails.
            static std::string Exec(const std::string &cmd) {
                std::array<char, 128> buffer;
                std::string result;
                std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
                if (!pipe) throw std::runtime_error("popen() failed!");

                while (!feof(pipe.get())) {
                    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
                        result += buffer.data();
                }

                return result;
            }

            static void CreateVirtualSerialPortPair() {
                std::cout << "Creating virtual serial port pair..." << std::endl;
                std::system("nohup sudo socat -d -d pty,raw,echo=0,link=/dev/ttyS10 pty,raw,echo=0,link=/dev/ttyS11 &");

                // Hacky! Since socat is detached, we have no idea at what point it has created
                // ttyS10 and ttyS11. Assume 1 second is long enough...
                std::this_thread::sleep_for(1s);
                std::system("sudo chmod a+rw /dev/ttyS10");
                std::system("sudo chmod a+rw /dev/ttyS11");
            }

            static void CloseSerialPorts() {
                // Dangerous! Kills all socat processes running
                // on computer
                std::system("sudo pkill socat");
            }

        };
    } // namespace CppLinuxSerial
} // namespace mn

#endif // #ifndef MN_CPP_LINUX_SERIAL_TEST_UTIL_H_
