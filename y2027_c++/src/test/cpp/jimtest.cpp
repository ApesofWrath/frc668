#include <gtest/gtest.h>
#include <subsystems/drivetrain/Constants.hpp>
#include <iostream>

using namespace Apes668;

class JimTest : public testing::Test {

};

TEST_F(JimTest, HelloWorld) {
    SwerveModuleCommonConstants smcc = SwerveModuleCommonConstants();
    std::cout << " SwerveModuleCommonConstants::drive_motor_gear_ratio - " << smcc.drive_motor_gear_ratio << std::endl;
    EXPECT_DOUBLE_EQ(0.0, 0.0);
};