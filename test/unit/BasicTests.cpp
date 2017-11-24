#include "gtest/gtest.h"

#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

namespace {

    class BasicTest : public ::testing::Test {
    protected:

        BasicTest() {
        }

        virtual ~BasicTest() {
        }
    };

    TEST_F(BasicTest, SimplePacket) {
        EXPECT_EQ(true, true);
    }

}  // namespace