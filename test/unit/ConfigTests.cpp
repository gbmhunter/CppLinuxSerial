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

    class ConfigTests : public ::testing::Test {
    protected:

        ConfigTests() {
        }

        virtual ~ConfigTests() {
        }

        std::string device0_ = "/dev/ttyS10";
        std::string device1_ = "/dev/ttyS11";
    };

    TEST_F(ConfigTests, BaudRateSetCorrectly) {
        SerialPort serialPort0(device0_, BaudRate::B_57600);
        serialPort0.Open();
        auto retVal = TestUtil::Exec("stty -a -F " + device0_);
        EXPECT_NE(std::string::npos, retVal.find("speed 57600 baud"));

        serialPort0.SetBaudRate(BaudRate::B_115200);
        retVal = TestUtil::Exec("stty -a -F " + device0_);
        EXPECT_NE(std::string::npos, retVal.find("speed 115200 baud"));
    }

    TEST_F(ConfigTests, CanonicalModeOff) {
        SerialPort serialPort0(device0_, BaudRate::B_57600);
        serialPort0.Open();
        auto retVal = TestUtil::Exec("stty -a -F " + device0_);
        EXPECT_NE(std::string::npos, retVal.find("-icanon"));
    }

    TEST_F(ConfigTests, EchoModeOff) {
        SerialPort serialPort0(device0_, BaudRate::B_57600);
        serialPort0.Open();
        auto retVal = TestUtil::Exec("stty -a -F " + device0_);
        EXPECT_NE(std::string::npos, retVal.find("-echo"));
    }

}  // namespace