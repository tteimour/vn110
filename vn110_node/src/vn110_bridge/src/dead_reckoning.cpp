#include <vn110_bridge/dead_reckoning.hpp>

DeadReckoning::DeadReckoning(rclcpp::Node* node) : node_(node)
{
    pub_vn_pos_   = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/dead_reckoning/vn110_position", 10);
    pub_px4_pos_  = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/dead_reckoning/px4_position", 10);
    pub_drift_    = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/dead_reckoning/drift", 10);
    pub_vel_      = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/dead_reckoning/velocity", 10);

    px4_sub_ = node_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position",
        rclcpp::QoS(10).best_effort(),
        std::bind(&DeadReckoning::px4Callback, this, std::placeholders::_1));
}

void DeadReckoning::initialize(double lat,
                               double lon,
                               double alt)
{
    origin_lat_ = lat * M_PI / 180.0;
    origin_lon_ = lon * M_PI / 180.0;
    origin_alt_ = alt;

    dr::radiiOfCurvature(origin_lat_, origin_Rn_, origin_Re_);

    vel_       = {0, 0, 0};
    pos_       = {0, 0, 0};
    px4_ned_   = {0, 0, 0};
    imu_count_ = 0;
    on_        = true;

    RCLCPP_INFO(node_->get_logger(),
        "Dead reckoning initialized at lat=%.7f  lon=%.7f  alt=%.2f", lat, lon, alt);
}

void DeadReckoning::stop()
{
    on_ = false;
    RCLCPP_INFO(node_->get_logger(), "Dead reckoning STOPPED");
}


void DeadReckoning::processDeltaVel(const rclcpp::Time& stamp,
                                                   double dvx,
                                                   double dvy,
                                                   double dvz,
                                                   double dt)
{
    if (!on_) return;
    if (dt <= 0.0 || dt > 1.0) return;

    // deltaVel is already in NED frame with gravity, bias, and Coriolis/earth-rate
    // compensated by the onboard coning/sculling algorithm (DeltaThetaVelConfig).
    dr::Vec3 vp = vel_;
    vel_[0] += dvx;
    vel_[1] += dvy;
    vel_[2] += dvz;

    for (int i = 0; i < 3; i++)
        pos_[i] += 0.5 * (vp[i] + vel_[i]) * dt;

    if (++imu_count_ % dr_publish_decimate_ != 0) return;


    auto st = stamp;

    auto vn_pos = geometry_msgs::msg::Vector3Stamped();
    vn_pos.header.stamp = st;
    vn_pos.header.frame_id = "dr_ned";
    vn_pos.vector.x = pos_[0];
    vn_pos.vector.y = pos_[1];
    vn_pos.vector.z = 0.0;
    pub_vn_pos_->publish(vn_pos);

    auto vm = geometry_msgs::msg::Vector3Stamped();
    vm.header.stamp = st;
    vm.header.frame_id = "dr_ned";
    vm.vector.x = vel_[0];
    vm.vector.y = vel_[1];
    vm.vector.z = vel_[2];
    pub_vel_->publish(vm);

    auto drift = geometry_msgs::msg::Vector3Stamped();
    drift.header.stamp = st;
    drift.header.frame_id = "dr_ned";
    drift.vector.x = pos_[0] - px4_ned_[0];
    drift.vector.y = pos_[1] - px4_ned_[1];
    drift.vector.z = 0.0;
    pub_drift_->publish(drift);
}

void DeadReckoning::px4Callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr m)
{
    if (!m->lat_lon_valid || !m->alt_valid) return;

    px4_lat_ = m->lat;
    px4_lon_ = m->lon;
    px4_alt_ = m->alt;
    px4_valid_ = true;

    if (!on_) return;

    double dlat = m->lat * M_PI / 180.0 - origin_lat_;
    double dlon = m->lon * M_PI / 180.0 - origin_lon_;
    px4_ned_[0] = dlat * (origin_Rn_ + origin_alt_);
    px4_ned_[1] = dlon * (origin_Re_ + origin_alt_) * std::cos(origin_lat_);
    px4_ned_[2] = -(m->alt - origin_alt_);

    auto st            = node_->now();

    auto pm            = geometry_msgs::msg::Vector3Stamped();
    pm.header.stamp    = st;
    pm.header.frame_id = "dr_ned";
    pm.vector.x        = px4_ned_[0];
    pm.vector.y        = px4_ned_[1];
    pm.vector.z        = 0.0;
    pub_px4_pos_->publish(pm);
}

