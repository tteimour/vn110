#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vectornav/Interface/Sensor.hpp>
#include <vectornav/Interface/Registers.hpp>
#include <vectornav/Interface/CompositeData.hpp>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <array>
#include <unordered_map>

class DeadReckoning;

using ParamMap = std::unordered_map<std::string, rclcpp::Parameter>;

using VpeBasicControl       = VN::Registers::Attitude::VpeBasicControl;
using VelAidingControl      = VN::Registers::VelocityAiding::VelAidingControl;
using RealTimeHsiControl    = VN::Registers::HardSoftIronEstimator::RealTimeHsiControl;
using RefModelConfig        = VN::Registers::WorldMagGravityModel::RefModelConfig;
using RefFrameRot           = VN::Registers::IMU::RefFrameRot;
using DeltaThetaVelConfig   = VN::Registers::IMU::DeltaThetaVelConfig;
using EstMagCal             = VN::Registers::HardSoftIronEstimator::EstMagCal;
using BinaryOutput1         = VN::Registers::System::BinaryOutput1;


class VN110Bridge : public rclcpp::Node
{
public:
    VN110Bridge();
    ~VN110Bridge();

private:
    VN::Sensor sensor_;
    std::string connected_port_;
    uint32_t connected_baudrate_ = 0;
    std::mutex sensor_mutex_;
    std::atomic<bool> running_{false};
    std::thread data_thread_;
    int ahrs_rate_;
    std::string frame_id_ = "vn110";

    bool velocity_aiding_enabled_ = true;
    double velocity_uncertainty_  = 0.1;
    bool vel_aid_active_ = false;
    std::atomic<float> latest_airspeed_{0.0f};

    void configureBinaryOutput();
    void dataProcessingLoop();
    void processCompositeData(const VN::CompositeData* cd);
    void printDeviceInfo();
    int getParamInt(const ParamMap& changed,
                    const std::string& name) const;
    bool getParamBool(const ParamMap& changed,
                      const std::string& name) const;
    double getParamDouble(const ParamMap& changed,
                          const std::string& name) const;
    int clampInt(const std::string& name,
                 int value,
                 int min_val,
                 int max_val);
    double clampDouble(const std::string& name,
                       double value,
                       double min_val,
                       double max_val);
    void writeVelAiding(bool enable);
    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter>& parameters);
    void applyRefFrameRotConfig(const ParamMap& changed);
    void applyVpeConfig(const ParamMap& changed);
    void applyHsiConfig(const ParamMap& changed);
    void applyVelocityAidingConfig(const ParamMap& changed);
    void applyInitialHeading(const ParamMap& changed);
    void applyWorldModelConfig(const ParamMap& changed);
    void applyDeltaThetaVelConfig();
    void publishPx4Euler(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void processAirspeed(const px4_msgs::msg::AirspeedValidated::SharedPtr msg);
    void pollHsiResults();
    void startHsiPolling();
    void stopHsiPolling();

    std::unique_ptr<DeadReckoning> dr_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr aided_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr aided_vel_sent_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr px4_euler_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr px4_gyro_deg_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vn110_gyro_deg_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr lin_accel_ned_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gyro_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr accel_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mag_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pres_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr aid_vel_thrsh_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr any_saturation_pub_;

    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated>::SharedPtr airspeed_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr write_settings_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_sensor_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restore_factory_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr known_mag_dist_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr known_accel_dist_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_baudrate_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr read_hsi_results_srv_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::TimerBase::SharedPtr hsi_poll_timer_;
    bool hsi_has_previous_{false};
    std::array<float, 12> hsi_prev_{};
    int hsi_converged_count_{0};
};
