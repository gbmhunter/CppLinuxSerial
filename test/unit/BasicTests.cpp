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

    TEST_F(BasicTests, BaudRateSetCorrectly) {
        SerialPort serialPort0(device0_, BaudRate::B_57600);
        serialPort0.Open();
        auto retVal = TestUtil::Exec("stty -a -F " + device0_);
        EXPECT_NE(std::string::npos, retVal.find("speed 57600 baud"));

        serialPort0.SetBaudRate(BaudRate::B_115200);
        retVal = TestUtil::Exec("stty -a -F " + device0_);
        EXPECT_NE(std::string::npos, retVal.find("speed 115200 baud"));
    }

    TEST_F(BasicTests, CanOpen) {
        SerialPort serialPort0(device0_, BaudRate::B_57600);
        serialPort0.Open();
    }

    TEST_F(BasicTests, ReadWrite) {
        SerialPort serialPort0(device0_, BaudRate::B_57600);
        serialPort0.Open();

        SerialPort serialPort1(device1_, BaudRate::B_57600);
        serialPort1.Open();

        serialPort0.Write("Hello");

        std::string readData;
        serialPort1.Read(readData);

        ASSERT_EQ("Hello", readData);
    }


    TEST_F(BasicTests, ReadWriteDiffBaudRates) {
        SerialPort serialPort0(device0_, BaudRate::B_9600);
        serialPort0.Open();

        SerialPort serialPort1(device1_, BaudRate::B_57600);
        serialPort1.Open();

        serialPort0.Write("Hello");

        std::string readData;
        serialPort1.Read(readData);

        ASSERT_EQ("Hello", readData);
    }

}  // namespace