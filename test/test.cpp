#include <gtest/gtest.h>
#include "../include/LaneDetector_test.hpp"


TEST(LaneTest, lane_detected) {
  EXPECT_EQ(testing_lanes(), 0);
}
