///
/// \file 			ConfigTests.cpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2017-11-24
/// \last-modified 	2017-11-24
/// \brief			Configuration tests for the SerialPort class.
/// \details
///					See README.rst in repo root dir for more info.

// System includes
#include "gtest/gtest.h"

// 3rd party includes
#include "CppLinuxSerial/SerialPort.hpp"

// User includes
#include "TestUtil.hpp"

using namespace mn::CppLinuxSerial;

namespace {

    // class ConfigTests : public ::testing::Test {
    // protected:

    //     ConfigTests() {
    //         serialPort_ = SerialPort(TestUtil::GetInstance().GetDevice0Name(), BaudRate::B_57600);
    //         serialPort_.Open();
    //         sttyOutput_ = TestUtil::GetInstance().Exec("stty -a -F " + TestUtil::GetInstance().GetDevice0Name());
    //     }

    //     virtual ~ConfigTests() {
    //     }

    //     SerialPort serialPort_;
    //     std::string sttyOutput_;
    // };

    // TEST_F(ConfigTests, BaudRateSetCorrectly) {
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("speed 57600 baud"));
    //     serialPort_.SetBaudRate(BaudRate::B_115200);
    //     sttyOutput_ = TestUtil::GetInstance().Exec("stty -a -F " + TestUtil::GetInstance().GetDevice0Name());
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("speed 115200 baud"));
    // }

    // //================================================================================================//
    // //======================================= LOCAL MODES (c_lflag) ==================================//
    // //================================================================================================//

    // TEST_F(ConfigTests, CanonicalModeOff) {
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("-icanon"));
    // }

    // TEST_F(ConfigTests, EchoModeOff) {
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("-echo"));
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("-echoe"));
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("-echonl"));
    // }

    // TEST_F(ConfigTests, InterruptQuitSuspCharsOff) {
    //     EXPECT_NE(std::string::npos, sttyOutput_.find("-isig"));
    // }

}  // namespace