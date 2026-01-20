#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>

#include "vff_avoidance/avoidance_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Case 1: No obstacles (All ranges are Infinity)
sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.header.frame_id = "laser_frame";
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0; // 16 Rays
  ret.range_min = 0.0f;
  ret.range_max = 10.0f;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());
  return ret;
}

// Case 2: Obstacle BEHIND (at -PI)
// Index 0 corresponds to -3.14 rads
sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  auto ret = get_scan_test_1(ts);
  ret.ranges.assign(16, 5.0f); // Default far
  ret.ranges[0] = 1.2f;        // Obstacle behind
  return ret;
}

// Case 3: Obstacle IN FRONT (at 0.0 rads)
// Index 8 corresponds to 0.0 rads (-PI + 8 * step)
sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  auto ret = get_scan_test_1(ts);
  ret.ranges.assign(16, 5.0f);
  ret.ranges[8] = 0.5f;        // Obstacle close in front
  return ret;
}

// Case 4: Obstacle to the RIGHT (at -PI/2)
// Index 4 corresponds to approx -1.57 rads
sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  auto ret = get_scan_test_1(ts);
  ret.ranges.assign(16, 5.0f);
  ret.ranges[4] = 0.5f;        // Obstacle to the right
  return ret;
}

// Case 5: Obstacle at ~45 Degrees LEFT (at +0.785 rads)
// Index 10 corresponds to approx 0.785 rads
sensor_msgs::msg::LaserScan get_scan_test_5(rclcpp::Time ts)
{
  auto ret = get_scan_test_1(ts);
  ret.ranges.assign(16, 5.0f);
  ret.ranges[10] = 0.3f;       // Obstacle very close
  return ret;
}

// TEST WRAPPER CLASS
// Allows access to protected methods of the main node
class AvoidanceNodeTest : public vff_avoidance_node::VffAvoidance
{
public:
  vff_avoidance_node::VFFVectors get_vff_test(const sensor_msgs::msg::LaserScan & scan)
  {
    return get_vff(scan);
  }
};


// UNIT TESTS (Logic Verification)

TEST(vff_tests, get_vff)
{
  auto node_avoidance = std::make_shared<AvoidanceNodeTest>();
  rclcpp::Time ts = node_avoidance->now();

  // --- Test 1: No Obstacles ---
  // Expectation: Only attractive force (1,0)
  auto res1 = node_avoidance->get_vff_test(get_scan_test_1(ts));
  ASSERT_EQ(res1.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_EQ(res1.repulsive, std::vector<float>({0.0f, 0.0f}));
  ASSERT_EQ(res1.result, std::vector<float>({1.0f, 0.0f}));

  // --- Test 2: Obstacle Behind ---
  // Expectation: Repulsive force pushes Forward (+X)
  auto res2 = node_avoidance->get_vff_test(get_scan_test_2(ts));
  ASSERT_GT(res2.repulsive[0], 0.0f); // Push forward
  ASSERT_NEAR(res2.repulsive[1], 0.0f, 0.1f); // No side push
  ASSERT_GT(res2.result[0], 1.0f);    // Result is stronger forward than 1.0

  // --- Test 3: Obstacle In Front ---
  // Expectation: Repulsive force pushes Backward (-X)
  auto res3 = node_avoidance->get_vff_test(get_scan_test_3(ts));
  ASSERT_LT(res3.repulsive[0], 0.0f); // Push backward
  // Result X should likely be negative (repulsion > attraction)
  ASSERT_LT(res3.result[0], 1.0f);

  // --- Test 4: Obstacle Right ---
  // Expectation: Repulsive force pushes Left (+Y)
  auto res4 = node_avoidance->get_vff_test(get_scan_test_4(ts));
  ASSERT_GT(res4.repulsive[1], 0.0f); // Push Left
  ASSERT_NEAR(res4.repulsive[0], 0.0f, 0.5f); // X push negligible
  // Robot should turn left (Positive Y in result)
  ASSERT_GT(res4.result[1], 0.5f);

  // --- Test 5: Obstacle 45 deg Left ---
  // Expectation: Repulsive force pushes Bottom-Right (-X, -Y)
  auto res5 = node_avoidance->get_vff_test(get_scan_test_5(ts));
  ASSERT_LT(res5.repulsive[0], 0.0f);
  ASSERT_LT(res5.repulsive[1], 0.0f);
  
  // Angle Checks
  float angle_rep = atan2(res5.repulsive[1], res5.repulsive[0]);
  // Repulsive vector should be in 3rd Quadrant (-PI to -PI/2)
  ASSERT_GT(angle_rep, -M_PI);
  ASSERT_LT(angle_rep, -M_PI_2);
}

// INTEGRATION TEST (Simulation Loop)
TEST(vff_tests, output_vels)
{
  auto node_avoidance = std::make_shared<AvoidanceNodeTest>();
  auto test_node = rclcpp::Node::make_shared("test_node");

  // Create infrastructure
  auto scan_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>("input_scan", 100);
  geometry_msgs::msg::Twist last_vel;
  auto vel_sub = test_node->create_subscription<geometry_msgs::msg::Twist>(
    "output_vel", 1, 
    [&last_vel](geometry_msgs::msg::Twist::SharedPtr msg) { last_vel = *msg; });

  ASSERT_EQ(vel_sub->get_publisher_count(), 1);
  ASSERT_EQ(scan_pub->get_subscription_count(), 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_avoidance);
  executor.add_node(test_node);
  rclcpp::Rate rate(30);

  // Run loop with "No Obstacles" scan
  auto start = node_avoidance->now();
  while (rclcpp::ok() && (node_avoidance->now() - start) < 1s) {
    scan_pub->publish(get_scan_test_1(node_avoidance->now()));
    executor.spin_some();
    rate.sleep();
  }

  // Expect standard forward velocity
  ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.05f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.05f);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}