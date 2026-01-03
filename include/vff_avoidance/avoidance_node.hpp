#ifndef VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_
#define VFF_AVOIDANCE__AVOIDANCE_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vff_avoidance_node
{
    
    struct VFFVectors{
        std::vector<float> attractive;
        std::vector<float> repulsive;
        std::vector<float> result;
    };

    class VffAvoidance : public rclcpp::Node
    {
        public:
            VffAvoidance();
        protected:
            VFFVectors get_vff(const sensor_msgs::msg::LaserScan &scan);
        private:
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

            rclcpp::TimerBase::SharedPtr timer_;

            sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
            void scan_callback(const sensor_msgs::msg::LaserScan::UniquePtr scan_msg);
            
            void control_cycle();

            visualization_msgs::msg::Marker make_marker(const std::vector<float> &vector, int id, float r, float g, float b);
            visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors &vff);
    };
} //namespace

#endif