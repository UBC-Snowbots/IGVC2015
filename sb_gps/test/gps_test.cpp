#include "../src/gps.cpp"
#include <gtest/gtest.h>

TEST (src,angleCorrection){
  EXPECT_EQ(0, sb_gps::createAngle(0.0,0.0));
  EXPECT_EQ(0, sb_gps::createAngle(180.0,180.0));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
