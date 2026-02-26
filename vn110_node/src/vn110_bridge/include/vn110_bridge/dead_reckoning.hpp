#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>
#include <array>
#include <atomic>

namespace dr 
{
using Vec3 = std::array<double, 3>;


inline constexpr double WGS84_A  = 6378137.0;
inline constexpr double WGS84_E2 = 0.00669437999014;

inline void radiiOfCurvature(double lat,
                             double& Rn,
                             double& Re)
{
    double s = std::sin(lat);
    double d = std::sqrt(1.0 - WGS84_E2 * s * s);
    Re = WGS84_A / d;
    Rn = WGS84_A * (1.0 - WGS84_E2) / (d * d * d);
}

}


class DeadReckoning 
{
public:
    explicit DeadReckoning(rclcpp::Node* node);

    void initialize(double lat, double lon, double alt);
    void stop();
    bool isRunning() const { return on_; }
    bool hasPx4Position() const { return px4_valid_; }

    void processDeltaVel(const rclcpp::Time& stamp,
                                        double dvx,
                                        double dvy,
                                        double dvz,
                                        double dt);



    double px4Lat() const { return px4_lat_; }
    double px4Lon() const { return px4_lon_; }
    double px4Alt() const { return px4_alt_; }

private:

    rclcpp::Node* node_;

    std::atomic<bool> on_{false};
    std::atomic<bool> px4_valid_{false};


    int    imu_count_           = 0; 
    int    dr_publish_decimate_ = 10;  // publish DR topics every Nth IMU cycle

    double origin_lat_ = 0, origin_lon_ = 0, origin_alt_ = 0;
    double origin_Rn_  = 0, origin_Re_  = 0;
    double px4_lat_    = 0, px4_lon_    = 0, px4_alt_    = 0;

    dr::Vec3 vel_{}, pos_{}, px4_ned_{};

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vn_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_px4_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_drift_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vel_;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr px4_sub_;

    void px4Callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr m);
};
