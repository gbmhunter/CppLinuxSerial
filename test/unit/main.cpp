///
/// \file 				main.cpp
/// \author 			Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \edited             n/a
/// \created			2017-11-24
/// \last-modified		2017-11-24
/// \brief 				Contains the main entry point for the unit tests.
/// \details
///		See README.md in root dir for more info.

// System includes
#include "gtest/gtest.h"

// User includes
#include "TestUtil.hpp"

using namespace mn::CppLinuxSerial;

class Environment : public testing::Environment {
public:
    virtual ~Environment() {}
    // Override this to define how to set up the environment.
    virtual void SetUp() {
        std::cout << __PRETTY_FUNCTION__ << " called." << std::endl;
        TestUtil::GetInstance().CreateVirtualSerialPortPair();
    }
    // Override this to define how to tear down the environment.
    virtual void TearDown() {
        TestUtil::GetInstance().CloseSerialPorts();
    }
};

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);

    // Create and register global test setup
    // (gtest takes ownership of pointer, do not delete manaully!)
    ::testing::AddGlobalTestEnvironment(new Environment);

  	return RUN_ALL_TESTS();
}