#include "gtest/gtest.h"

#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

namespace {

    class BasicTests : public ::testing::Test {
    protected:

        BasicTests() {
        }

        virtual ~BasicTests() {
        }
    };

    TEST_F(BasicTests, CanBeConstructed) {
        SerialPort serialPort;
        EXPECT_EQ(true, true);
    }

    TEST_F(BasicTests, CanOpen) {
        SerialPort serialPort;
        serialPort.Open();
        EXPECT_EQ(true, true);
    }

}  // namespace