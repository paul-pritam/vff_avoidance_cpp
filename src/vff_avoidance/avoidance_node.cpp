#include "vff_avoidance/avoidance_node.hpp"

#include <cmath>
#include <algorithm>

namespace vff_avoidance_node{
    VffAvoidance::VffAvoidance(): Node("avoidance_vff"){
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::SensorDataQoS(), std::bind(&VffAvoidance::scan_callback, this, std::placeholders::_1));
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel",10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug",10);
        timer_= create_wall_timer(std::chrono::milliseconds(50),std::bind(&VffAvoidance::control_cycle,this));
    }

    void VffAvoidance::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr scan_msg){
        last_scan_= std::move(scan_msg);
    }

    VFFVectors VffAvoidance::get_vff(const sensor_msgs::msg::LaserScan &scan){
        const float OBSTACLE_DIST = 1.5;
        VFFVectors vff;

        vff.attractive = {1.0,0.0}; //Attractive Vector : constant pull forward (x=1)

        float min_dist = OBSTACLE_DIST;
        float angle_min_dist = 0.0;

        //scan +/-3o degrees for obstacle
        int half_window = std::abs(0.52/scan.angle_increment); //number of rays
        int center = scan.ranges.size()/2;

        for (int i = center-half_window; i <= (center+half_window); i++){
            if (i >= 0 && i < (int)scan.ranges.size() ){
                if (scan.ranges[i] < min_dist && scan.ranges[i] > scan.range_min){
                    min_dist = scan.ranges[i];
                    angle_min_dist = scan.angle_min + (i * scan.angle_increment);
                }
            }
        }
        vff.repulsive = {0.0,0.0};
        if (min_dist < OBSTACLE_DIST){
            float repulse_mag = (OBSTACLE_DIST - min_dist)/OBSTACLE_DIST; //more error more repulse
            //direction is opposite to the obstacle
            vff.repulsive[0] = -cos(angle_min_dist) * repulse_mag;
            vff.repulsive[1] = -sin(angle_min_dist) * repulse_mag;
        }
        vff.result = {vff.attractive[0]+vff.repulsive[0],vff.attractive[1]+vff.repulsive[1]};
        return vff;
    }

    void VffAvoidance::control_cycle(){
        if (last_scan_ == nullptr){
            return;
        }

        VFFVectors vff = get_vff(*last_scan_);
        auto markers = get_debug_vff(vff);
        marker_pub_ -> publish(markers);

        geometry_msgs::msg::Twist vel;
        float angle = atan2(vff.result[1],vff.result[0]);
        float mag = sqrt(pow(vff.result[0], 2) + pow(vff.result[1],2));

        vel.linear.x = std::clamp(mag*0.3f, 0.0f, 0.3f);
        vel.angular.z = std::clamp(angle, -0.5f, 0.5f);
        vel_pub_->publish(vel);
    }
}