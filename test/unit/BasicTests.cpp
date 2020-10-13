///
/// \file 			BasicTests.cpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2017-11-24
/// \last-modified 	2019-05-30
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

        BasicTests() {
        }

        virtual ~BasicTests() {
        }

        // std::string device0Name_ = TestUtil::GetInstance().GetDevice0Name();
        // std::string device1Name_ = TestUtil::GetInstance().GetDevice1Name();
    };

    TEST_F(BasicTests, CanBeConstructed) {
        SerialPort serialPort;
        EXPECT_EQ(true, true);
    }

    // TEST_F(BasicTests, CanOpen) {
    //     SerialPort serialPort0(device0Name_, BaudRate::B_57600);
    //     serialPort0.Open();
    // }

    // TEST_F(BasicTests, ReadWrite) {
    //     SerialPort serialPort0(device0Name_, BaudRate::B_57600);
    //     serialPort0.Open();

    //     SerialPort serialPort1(device1Name_, BaudRate::B_57600);
    //     serialPort1.Open();

    //     serialPort0.Write("Hello");

    //     std::string readData;
    //     serialPort1.Read(readData);

    //     ASSERT_EQ("Hello", readData);
    // }

    // TEST_F(BasicTests, ReadWriteDiffBaudRates) {
    //     SerialPort serialPort0(device0Name_, BaudRate::B_9600);
    //     serialPort0.Open();

    //     SerialPort serialPort1(device1Name_, BaudRate::B_57600);
    //     serialPort1.Open();

    //     serialPort0.Write("Hello");

    //     std::string readData;
    //     serialPort1.Read(readData);

    //     ASSERT_EQ("Hello", readData);
    // }

    // TEST_F(BasicTests, SetTimeoutCorrectly) {
    //     SerialPort serialPort0(device0Name_, BaudRate::B_57600);
    //     serialPort0.SetTimeout(-1); // Infinite timeout
    //     serialPort0.Open();

    //     SerialPort serialPort1(device1Name_, BaudRate::B_57600);
    //     serialPort1.Open();

    //     serialPort0.Write("Hello");

    //     std::string readData;
    //     serialPort1.Read(readData);

    //     ASSERT_EQ("Hello", readData);
    // }

    // TEST_F(BasicTests, SetTimeoutIncorrectly) {
    //     SerialPort serialPort0(device0Name_, BaudRate::B_57600);
    //     serialPort0.Open();
    //     EXPECT_THROW(serialPort0.SetTimeout(-1), mn::CppLinuxSerial::Exception);
    // }

}  // namespace