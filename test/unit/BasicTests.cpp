///
/// \file 			BasicTests.cpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2017-11-24
/// \last-modified 	2017-11-24
/// \brief			Basic tests for the SerialPort class.
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

    class BasicTests : public ::testing::Test {
    protected:

        static void SetUpTestCase() {
            GetTestUtil().CreateVirtualSerialPortPair();
        }

        static void TearDownTestCase() {
            std::cout << "Destroying virtual serial ports..." << std::endl;
            GetTestUtil().CloseSerialPorts();
        }

        static TestUtil& GetTestUtil() {
            static TestUtil testUtil;
            return testUtil;
        }


        BasicTests() {
        }

        virtual ~BasicTests() {
        }

        std::string device0_ = "/dev/ttyS10";
        std::string device1_ = "/dev/ttyS11";
    };

    TEST_F(BasicTests, CanBeConstructed) {
        SerialPort serialPort;
        EXPECT_EQ(true, true);
    }

    TEST_F(BasicTests, CanOpen) {
        SerialPort serialPort0(device0_, BaudRate::b57600);
        serialPort0.Open();
    }

    TEST_F(BasicTests, ReadWrite) {
        SerialPort serialPort0(device0_, BaudRate::b57600);
        serialPort0.Open();

        SerialPort serialPort1(device1_, BaudRate::b57600);
        serialPort1.Open();

    }

}  // namespace