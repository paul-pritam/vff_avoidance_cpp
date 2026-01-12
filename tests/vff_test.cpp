#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>

#include "vff_avoidance/avoidance_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

// Create a Test Wrapper to access protected methods
// Inherits from the class defined in your implementation (vff_avoidance_node::VffAvoidance)
class AvoidanceNodeWrapper : public vff_avoidance_node::VffAvoidance
{
public:
  // Helper to access the protected get_vff method
  vff_avoidance_node::VFFVectors compute_vff(const sensor_msgs::msg::LaserScan & scan)
  {
    return get_vff(scan);
  }
};

class VFFTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);
    // Instantiate the wrapper class
    node = std::make_shared<AvoidanceNodeWrapper>();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  // DECLARE MEMBER VARIABLE :
  std::shared_ptr<AvoidanceNodeWrapper> node;
};

// TEST 1: No obstacles (All ranges are infinity)
TEST_F(VFFTest, no_obstacles) {
  sensor_msgs::msg::LaserScan scan;
  scan.ranges.assign(100, std::numeric_limits<float>::infinity());
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = (2.0 * M_PI) / 100.0;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  auto vff = node->compute_vff(scan);

  // Expect standard forward force (1.0, 0.0) with no repulsion
  EXPECT_NEAR(vff.result[0], 1.0, 1e-5);
  EXPECT_NEAR(vff.result[1], 0.0, 1e-5);
  EXPECT_EQ(vff.repulsive[0], 0.0f);
}

// TEST 2: Obstacle directly in front
TEST_F(VFFTest, obstacle_in_front) {
  sensor_msgs::msg::LaserScan scan;
  scan.ranges.assign(100, 5.0);
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment = (2.0 * M_PI) / 100.0;
  scan.range_min = 0.1;
  scan.range_max = 10.0;

  // Place obstacle 0.5m in front
  scan.ranges[50] = 0.5f;

  auto vff = node->compute_vff(scan);

  // Repulsive force in X should be negative (pushing away from front)
  EXPECT_LT(vff.repulsive[0], 0.0);
  // Resultant X should be less than the original attractive 1.0
  EXPECT_LT(vff.result[0], 1.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
