#include <vn110_bridge/vn110_bridge.hpp>
#include <vn110_bridge/dead_reckoning.hpp>

VN110Bridge::VN110Bridge() : Node("vn110_bridge")
{
    declare_parameter<std::string>("port");
    declare_parameter<int>("baudrate");
    declare_parameter<int>("ahrs_rate");

    declare_parameter<int>("vpe_heading_mode");
    declare_parameter<int>("vpe_filtering_mode");
    declare_parameter<int>("vpe_tuning_mode");

    declare_parameter<int>("hsi_mode");
    declare_parameter<bool>("hsi_apply_compensation");
    declare_parameter<int>("hsi_converge_rate");
    declare_parameter<double>("hsi_poll_interval");
    declare_parameter<double>("hsi_convergence_threshold");

    declare_parameter<bool>("velocity_aiding_enabled");
    declare_parameter<double>("velocity_uncertainty");
    declare_parameter<double>("velocity_aiding_threshold");

    declare_parameter("initial_heading", std::numeric_limits<double>::quiet_NaN());

    declare_parameter<bool>("world_model_enabled");
    declare_parameter<double>("world_model_latitude");
    declare_parameter<double>("world_model_longitude");
    declare_parameter<double>("world_model_altitude");

    declare_parameter<double>("ref_frame_rot_00");
    declare_parameter<double>("ref_frame_rot_01");
    declare_parameter<double>("ref_frame_rot_02");
    declare_parameter<double>("ref_frame_rot_10");
    declare_parameter<double>("ref_frame_rot_11");
    declare_parameter<double>("ref_frame_rot_12");
    declare_parameter<double>("ref_frame_rot_20");
    declare_parameter<double>("ref_frame_rot_21");
    declare_parameter<double>("ref_frame_rot_22");

    declare_parameter<bool>("deadreckoning_enabled");

    std::string port  = get_parameter("port").as_string();
    int baudrate      = get_parameter("baudrate").as_int();
    ahrs_rate_        = get_parameter("ahrs_rate").as_int();

    RCLCPP_INFO(get_logger(), "Connecting to VN-110 on %s @ %d baud...", port.c_str(), baudrate);

    VN::Sensor::BaudRate vnBaud = static_cast<VN::Sensor::BaudRate>(baudrate);
    VN::Error err = sensor_.connect(port, vnBaud);

    if (err != VN::Error::None || !sensor_.verifySensorConnectivity()) {
        RCLCPP_WARN(get_logger(), "Failed at %d baud, trying auto-connect...", baudrate);
        sensor_.disconnect();
        err = sensor_.autoConnect(port);
    }

    if (err != VN::Error::None) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to VN-110 on %s", port.c_str());
        return;
    }

    connected_port_ = port;
    VN::Registers::System::BaudRate br;
    br.serialPort = VN::Registers::System::BaudRate::SerialPort::Serial1;
    if (sensor_.readRegister(&br) == VN::Error::None && br.baudRate.has_value()) {
        connected_baudrate_ = static_cast<uint32_t>(br.baudRate.value());
    }

    RCLCPP_INFO(get_logger(), "  VN-110 CONNECTED");
    RCLCPP_INFO(get_logger(), "  Port: %s @ %u baud", connected_port_.c_str(), connected_baudrate_);
    RCLCPP_INFO(get_logger(), "  AHRS Rate: %d Hz (Binary Output)", ahrs_rate_);

    printDeviceInfo();

    applyRefFrameRotConfig({});
    applyVpeConfig({});
    applyHsiConfig({});
    applyVelocityAidingConfig({});
    applyWorldModelConfig({});
    applyDeltaThetaVelConfig();
    configureBinaryOutput();

    imu_pub_            = create_publisher<sensor_msgs::msg::Imu>("/vn110/imu", 10);
    euler_pub_          = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vn110/euler", 10);
    aided_vel_pub_      = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vn110/aided_velocity", 10);
    aided_vel_sent_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vn110/aided_velocity_sent", 10);
    px4_euler_pub_      = create_publisher<geometry_msgs::msg::Vector3Stamped>("/fmu/out/vehicle_attitude_euler", 10);
    px4_gyro_deg_pub_   = create_publisher<geometry_msgs::msg::Vector3Stamped>("/fmu/out/gyro_deg", 10);
    vn110_gyro_deg_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vn110/gyro_deg", 10);
    lin_accel_ned_pub_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vn110/lin_accel_ned", 10);

    gyro_status_pub_    = create_publisher<std_msgs::msg::UInt8>("/vn110/status/gyro", 10);
    accel_status_pub_   = create_publisher<std_msgs::msg::UInt8>("/vn110/status/accel", 10);
    mag_status_pub_     = create_publisher<std_msgs::msg::UInt8>("/vn110/status/mag", 10);
    pres_status_pub_    = create_publisher<std_msgs::msg::UInt8>("/vn110/status/pressure", 10);
    aid_vel_thrsh_pub_  = create_publisher<std_msgs::msg::UInt8>("/vn110/status/aid_velocity_threshold", 10);
    any_saturation_pub_ = create_publisher<std_msgs::msg::Bool>("/vn110/status/any_saturation", 10);

    dr_ = std::make_unique<DeadReckoning>(this);

    auto qos = rclcpp::QoS(10).best_effort();

    sensor_combined_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined", qos,
        [this](const px4_msgs::msg::SensorCombined::SharedPtr msg) {
            geometry_msgs::msg::Vector3Stamped gyro_deg;
            gyro_deg.header.stamp = now();
            gyro_deg.header.frame_id = "fmu";
            gyro_deg.vector.x = msg->gyro_rad[0] * 180.0 / M_PI;
            gyro_deg.vector.y = msg->gyro_rad[1] * 180.0 / M_PI;
            gyro_deg.vector.z = msg->gyro_rad[2] * 180.0 / M_PI;
            px4_gyro_deg_pub_->publish(gyro_deg);
        });

    airspeed_sub_ = create_subscription<px4_msgs::msg::AirspeedValidated>(
        "/fmu/out/airspeed_validated", qos,
        [this](const px4_msgs::msg::AirspeedValidated::SharedPtr msg) { processAirspeed(msg); });

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) { publishPx4Euler(msg); });

    write_settings_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vn110/write_settings",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (sensor_.writeSettings() == VN::Error::None) {
                response->success = true;
                response->message = "Settings saved to flash";
                RCLCPP_INFO(get_logger(), "Settings saved to flash");
            } else {
                response->success = false;
                response->message = "Failed to save settings";
            }
        });

    reset_sensor_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vn110/reset_sensor",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (sensor_.reset() == VN::Error::None) {
                response->success = true;
                response->message = "Sensor reset";
                RCLCPP_WARN(get_logger(), "Sensor reset - unsaved settings lost");
            } else {
                response->success = false;
                response->message = "Failed to reset sensor";
            }
        });

    restore_factory_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vn110/restore_factory",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (sensor_.restoreFactorySettings() == VN::Error::None) {
                response->success = true;
                response->message = "Factory settings restored";
                RCLCPP_WARN(get_logger(), "Factory settings restored");
            } else {
                response->success = false;
                response->message = "Failed to restore factory settings";
            }
        });

    known_mag_dist_srv_ = create_service<std_srvs::srv::SetBool>(
        "/vn110/known_mag_disturbance",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            auto state = request->data ? VN::KnownMagneticDisturbance::State::Present
                                       : VN::KnownMagneticDisturbance::State::NotPresent;
            if (sensor_.knownMagneticDisturbance(state) == VN::Error::None) {
                response->success = true;
                response->message = request->data ? "Magnetic disturbance: PRESENT"
                                                  : "Magnetic disturbance: NOT PRESENT";
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            } else {
                response->success = false;
                response->message = "Failed to set magnetic disturbance state";
            }
        });

    known_accel_dist_srv_ = create_service<std_srvs::srv::SetBool>(
        "/vn110/known_accel_disturbance",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            auto state = request->data ? VN::KnownAccelerationDisturbance::State::Present
                                       : VN::KnownAccelerationDisturbance::State::NotPresent;
            if (sensor_.knownAccelerationDisturbance(state) == VN::Error::None) {
                response->success = true;
                response->message = request->data ? "Acceleration disturbance: PRESENT"
                                                  : "Acceleration disturbance: NOT PRESENT";
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            } else {
                response->success = false;
                response->message = "Failed to set acceleration disturbance state";
            }
        });

    change_baudrate_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vn110/set_baudrate_921600",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);

            auto err = sensor_.changeBaudRate(VN::Sensor::BaudRate::Baud921600);
            if (err != VN::Error::None) {
                response->success = false;
                response->message = "Failed to change baud rate: " + std::to_string(static_cast<int>(err));
                return;
            }

            if (sensor_.writeSettings() != VN::Error::None) {
                response->success = false;
                response->message = "Baud rate changed but failed to save to flash";
                return;
            }

            response->success = true;
            response->message = "Baud rate changed to 921600 and saved to flash.";
            RCLCPP_INFO(get_logger(), "Baud rate changed to 921600 and saved");
        });

    read_hsi_results_srv_ = create_service<std_srvs::srv::Trigger>(
        "/vn110/read_hsi_results",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            EstMagCal est;
            if (sensor_.readRegister(&est) != VN::Error::None) {
                response->success = false;
                response->message = "Failed to read Register 47";
                return;
            }
            char buf[512];
            std::snprintf(buf, sizeof(buf),
                "mag_cal_gain_00: %.6f\n"
                "mag_cal_gain_01: %.6f\n"
                "mag_cal_gain_02: %.6f\n"
                "mag_cal_gain_10: %.6f\n"
                "mag_cal_gain_11: %.6f\n"
                "mag_cal_gain_12: %.6f\n"
                "mag_cal_gain_20: %.6f\n"
                "mag_cal_gain_21: %.6f\n"
                "mag_cal_gain_22: %.6f\n"
                "mag_cal_bias_x: %.6f\n"
                "mag_cal_bias_y: %.6f\n"
                "mag_cal_bias_z: %.6f",
                est.magGain00, est.magGain01, est.magGain02,
                est.magGain10, est.magGain11, est.magGain12,
                est.magGain20, est.magGain21, est.magGain22,
                est.magBiasX, est.magBiasY, est.magBiasZ);
            response->success = true;
            response->message = buf;
            RCLCPP_INFO(get_logger(), "HSI Results (Register 47) - paste into YAML:\n%s", buf);
        });

    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&VN110Bridge::parameterCallback, this, std::placeholders::_1));

    running_ = true;
    data_thread_ = std::thread(&VN110Bridge::dataProcessingLoop, this);

    RCLCPP_INFO(get_logger(), "VN110 bridge + dead reckoning ready. Toggle deadreckoning_enabled to start/reset.");
}

VN110Bridge::~VN110Bridge()
{
    running_ = false;
    if (data_thread_.joinable()) {
        data_thread_.join();
    }
    sensor_.disconnect();
}


void VN110Bridge::configureBinaryOutput()
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    int divisor = 800 / ahrs_rate_;
    if (divisor < 1) divisor = 1;
    if (divisor > 800) divisor = 800;

    BinaryOutput1 bo1;

    BinaryOutput1::AsyncMode asyncMode;
    asyncMode.serial1 = 1;
    asyncMode.serial2 = 0;
    bo1.asyncMode = asyncMode;
    bo1.rateDivisor = static_cast<uint16_t>(divisor);

    bo1.common.quaternion = 1;
    bo1.common.angularRate = 1;
    bo1.common.accel = 1;
    bo1.common.ypr = 1;

    bo1.imu.imuStatus = 1;
    bo1.imu.sensSat = 1;
    bo1.imu.deltaTheta = 1;

    bo1.attitude.linAccelNed = 1;

    if (sensor_.writeRegister(&bo1) == VN::Error::None) {
        RCLCPP_INFO(get_logger(), "Binary Output configured: %d Hz (divisor=%d)", 800/divisor, divisor);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to configure Binary Output");
    }
}


void VN110Bridge::dataProcessingLoop()
{
    while (running_ && rclcpp::ok()) {
        auto cd = sensor_.getNextMeasurement();
        if (!cd) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        processCompositeData(cd.get());
    }
}


void VN110Bridge::processCompositeData(const VN::CompositeData* cd)
{
    auto stamp = now();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id_;

    if (cd->attitude.quaternion.has_value()) {
        const auto& q = cd->attitude.quaternion.value();
        msg.orientation.x = q.vector[0];
        msg.orientation.y = q.vector[1];
        msg.orientation.z = q.vector[2];
        msg.orientation.w = q.scalar;


    }

    if (cd->imu.angularRate.has_value()) {
        const auto& w = cd->imu.angularRate.value();
        msg.angular_velocity.x = w[0];
        msg.angular_velocity.y = w[1];
        msg.angular_velocity.z = w[2];

        geometry_msgs::msg::Vector3Stamped gyro_deg;
        gyro_deg.header   = msg.header;
        gyro_deg.vector.x = w[0] * 180.0 / M_PI;
        gyro_deg.vector.y = w[1] * 180.0 / M_PI;
        gyro_deg.vector.z = w[2] * 180.0 / M_PI;
        vn110_gyro_deg_pub_->publish(gyro_deg);
    }

    if (cd->imu.accel.has_value()) {
        const auto& a = cd->imu.accel.value();
        msg.linear_acceleration.x = a[0];
        msg.linear_acceleration.y = a[1];
        msg.linear_acceleration.z = a[2];
    }

    imu_pub_->publish(msg);

    if (cd->attitude.ypr.has_value()) {
        const auto& ypr = cd->attitude.ypr.value();
        geometry_msgs::msg::Vector3Stamped euler;
        euler.header   = msg.header;
        euler.vector.x = ypr.roll;
        euler.vector.y = ypr.pitch;
        euler.vector.z = ypr.yaw;
        euler_pub_->publish(euler);
    }

    if (cd->imu.imuStatus.has_value()) {
        uint16_t status = static_cast<uint16_t>(cd->imu.imuStatus.value());
        std_msgs::msg::UInt8 gyro_st, accel_st, mag_st, pres_st;
        gyro_st.data  = (status >> 0) & 0x03;
        accel_st.data = (status >> 2) & 0x03;
        mag_st.data   = (status >> 4) & 0x03;
        pres_st.data  = (status >> 6) & 0x03;
        gyro_status_pub_->publish(gyro_st);
        accel_status_pub_->publish(accel_st);
        mag_status_pub_->publish(mag_st);
        pres_status_pub_->publish(pres_st);
    }

    if (cd->imu.sensSat.has_value()) {
        uint16_t sat = cd->imu.sensSat.value();
        std_msgs::msg::Bool sat_msg;
        sat_msg.data = (sat != 0);
        any_saturation_pub_->publish(sat_msg);
    }

    if (cd->attitude.linAccelNed.has_value() && cd->imu.deltaTheta.has_value()) {
        const auto& la = cd->attitude.linAccelNed.value();
        double dt = cd->imu.deltaTheta.value().deltaTime;

        geometry_msgs::msg::Vector3Stamped la_msg;
        la_msg.header   = msg.header;
        la_msg.vector.x = la[0];
        la_msg.vector.y = la[1];
        la_msg.vector.z = la[2];
        lin_accel_ned_pub_->publish(la_msg);

        dr_->processDeltaVel(stamp, la[0] * dt, la[1] * dt, la[2] * dt, dt);
    }
}


void VN110Bridge::printDeviceInfo()
{
    VN::Registers::System::Model model;
    if (sensor_.readRegister(&model) == VN::Error::None)
        RCLCPP_INFO(get_logger(), "Model: %s", model.model.c_str());

    VN::Registers::System::Serial serial;
    if (sensor_.readRegister(&serial) == VN::Error::None)
        RCLCPP_INFO(get_logger(), "Serial: %u", serial.serialNum);

    VN::Registers::System::FwVer fwVer;
    if (sensor_.readRegister(&fwVer) == VN::Error::None)
        RCLCPP_INFO(get_logger(), "Firmware: %s", fwVer.fwVer.c_str());
}


int VN110Bridge::clampInt(const std::string& name,
                                          int value,
                                          int min_val,
                                          int max_val)
{
    if (value < min_val || value > max_val) {
        int clamped = std::clamp(value, min_val, max_val);
        RCLCPP_WARN(get_logger(), "Parameter '%s' value %d out of bounds [%d, %d], clamping to %d",
            name.c_str(), value, min_val, max_val, clamped);
        return clamped;
    }
    return value;
}

double VN110Bridge::clampDouble(const std::string& name,
                                double value,
                                double min_val,
                                double max_val)
{
    if (value < min_val || value > max_val) {
        double clamped = std::clamp(value, min_val, max_val);
        RCLCPP_WARN(get_logger(), "Parameter '%s' value %.3f out of bounds [%.3f, %.3f], clamping to %.3f",
            name.c_str(), value, min_val, max_val, clamped);
        return clamped;
    }
    return value;
}

int VN110Bridge::getParamInt(const ParamMap& changed,
                             const std::string& name) const
{
    return changed.count(name) ? changed.at(name).as_int() : get_parameter(name).as_int();
}

bool VN110Bridge::getParamBool(const ParamMap& changed,
                               const std::string& name) const
{
    return changed.count(name) ? changed.at(name).as_bool() : get_parameter(name).as_bool();
}

double VN110Bridge::getParamDouble(const ParamMap& changed,
                                   const std::string& name) const
{
    return changed.count(name) ? changed.at(name).as_double() : get_parameter(name).as_double();
}

void VN110Bridge::writeVelAiding(bool enable)
{
    VelAidingControl ctrl;
    ctrl.velAidEnable = enable ? VelAidingControl::VelAidEnable::Enable
                               : VelAidingControl::VelAidEnable::Disable;
    ctrl.velUncertTuning = static_cast<float>(velocity_uncertainty_);
    ctrl.resv = 0.01f;
    sensor_.writeRegister(&ctrl);
}


rcl_interfaces::msg::SetParametersResult VN110Bridge::parameterCallback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::unordered_map<std::string, rclcpp::Parameter> changed;
    for (const auto& p : parameters) {
        changed[p.get_name()] = p;
    }

    bool vpe_basic_changed       = false;
    bool vel_aiding_changed      = false;
    bool hsi_changed             = false;
    bool rate_changed            = false;
    bool initial_heading_changed = false;
    bool world_model_changed     = false;

    for (const auto& [name, param] : changed) {
        if (name == "vpe_heading_mode" || name == "vpe_filtering_mode" || name == "vpe_tuning_mode") {
            vpe_basic_changed = true;
        } else if (name.find("velocity_") == 0) {
            vel_aiding_changed = true;
        } else if (name.find("hsi_") == 0) {
            hsi_changed = true;
        } else if (name == "ahrs_rate") {
            rate_changed = true;
        } else if (name == "initial_heading") {
            initial_heading_changed = true;
        } else if (name.find("world_model_") == 0) {
            world_model_changed = true;
        }

        if (name == "deadreckoning_enabled") {
            bool val = param.as_bool();
            if (val && !dr_->isRunning()) {
                if (dr_->hasPx4Position())
                {
                    dr_->initialize(dr_->px4Lat(), dr_->px4Lon(), dr_->px4Alt());
                }
                else {
                    RCLCPP_WARN(get_logger(), "No PX4 position available, DR starting with origin (0,0,0)");
                    dr_->initialize(0.0, 0.0, 0.0);
                }
                dr_->initialize(dr_->px4Lat(), dr_->px4Lon(), dr_->px4Alt());
            } else if (!val && dr_->isRunning()) {
                dr_->stop();
            }
        }
    }

    if (vpe_basic_changed) applyVpeConfig(changed);
    if (hsi_changed) applyHsiConfig(changed);
    if (vel_aiding_changed) applyVelocityAidingConfig(changed);
    if (initial_heading_changed) applyInitialHeading(changed);
    if (world_model_changed) applyWorldModelConfig(changed);
    if (rate_changed) {
        int new_rate = changed.count("ahrs_rate") ? changed.at("ahrs_rate").as_int() : get_parameter("ahrs_rate").as_int();
        new_rate = clampInt("ahrs_rate", new_rate, 1, 400);
        ahrs_rate_ = new_rate;
        configureBinaryOutput();
    }

    return result;
}


void VN110Bridge::applyRefFrameRotConfig(const ParamMap& changed)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    RefFrameRot rfr;
    rfr.rfr00 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_00"));
    rfr.rfr01 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_01"));
    rfr.rfr02 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_02"));
    rfr.rfr10 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_10"));
    rfr.rfr11 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_11"));
    rfr.rfr12 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_12"));
    rfr.rfr20 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_20"));
    rfr.rfr21 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_21"));
    rfr.rfr22 = static_cast<float>(getParamDouble(changed, "ref_frame_rot_22"));

    if (sensor_.writeRegister(&rfr) == VN::Error::None) {
        RCLCPP_INFO(get_logger(), "Reference Frame Rotation applied");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply Reference Frame Rotation");
    }
}

void VN110Bridge::applyVpeConfig(const ParamMap& changed)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    int heading_mode   = clampInt("vpe_heading_mode", getParamInt(changed, "vpe_heading_mode"), 0, 2);
    int filtering_mode = clampInt("vpe_filtering_mode", getParamInt(changed, "vpe_filtering_mode"), 0, 1);
    int tuning_mode    = clampInt("vpe_tuning_mode", getParamInt(changed, "vpe_tuning_mode"), 0, 1);

    VpeBasicControl vpe;
    vpe.resv           = 1;
    vpe.headingMode    = static_cast<VpeBasicControl::HeadingMode>(heading_mode);
    vpe.filteringMode  = static_cast<VpeBasicControl::FilteringMode>(filtering_mode);
    vpe.tuningMode     = static_cast<VpeBasicControl::TuningMode>(tuning_mode);

    if (sensor_.writeRegister(&vpe) == VN::Error::None) {
        const char* heading_str[] = {"Absolute", "Relative", "Indoor"};
        const char* filtering_str[] = {"Unfiltered", "AdaptivelyFiltered"};
        const char* tuning_str[] = {"Static", "Adaptive"};
        RCLCPP_INFO(get_logger(), "VPE: Heading=%s, Filtering=%s, Tuning=%s",
            heading_str[heading_mode], filtering_str[filtering_mode], tuning_str[tuning_mode]);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply VPE configuration");
    }
}

void VN110Bridge::applyVelocityAidingConfig(const ParamMap& changed)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    bool enabled       = getParamBool(changed, "velocity_aiding_enabled");
    double uncertainty = clampDouble("velocity_uncertainty", getParamDouble(changed, "velocity_uncertainty"), 0.001, 10.0);

    velocity_aiding_enabled_ = enabled;
    velocity_uncertainty_    = uncertainty;

    VelAidingControl ctrl;
    ctrl.velAidEnable = enabled ? VelAidingControl::VelAidEnable::Enable
                                : VelAidingControl::VelAidEnable::Disable;
    ctrl.velUncertTuning = static_cast<float>(uncertainty);
    ctrl.resv = 0.01f;

    if (sensor_.writeRegister(&ctrl) == VN::Error::None) {
        RCLCPP_INFO(get_logger(), "Velocity Aiding: %s, Uncertainty=%.2f m/s",
            enabled ? "ENABLED" : "DISABLED", uncertainty);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply velocity aiding configuration");
    }
}

void VN110Bridge::applyHsiConfig(const ParamMap& changed)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    int mode          = clampInt("hsi_mode", getParamInt(changed, "hsi_mode"), 0, 2);
    bool apply_comp   = getParamBool(changed, "hsi_apply_compensation");
    int converge_rate = clampInt("hsi_converge_rate", getParamInt(changed, "hsi_converge_rate"), 1, 5);
    double conv_thresh = clampDouble("hsi_convergence_threshold",
        getParamDouble(changed, "hsi_convergence_threshold"), 1e-8, 1.0);

    RealTimeHsiControl hsi;
    hsi.mode              = static_cast<RealTimeHsiControl::Mode>(mode);
    hsi.applyCompensation = apply_comp ? RealTimeHsiControl::ApplyCompensation::Enable
                                       : RealTimeHsiControl::ApplyCompensation::Disable;
    hsi.convergeRate      = static_cast<uint8_t>(converge_rate);

    if (sensor_.writeRegister(&hsi) == VN::Error::None) {
        const char* mode_str[] = {"Off", "Run", "Reset"};
        RCLCPP_INFO(get_logger(), "HSI: Mode=%s, ApplyComp=%s, ConvergeRate=%d, ConvThreshold=%.8f",
            mode_str[mode], apply_comp ? "Yes" : "No", converge_rate, conv_thresh);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply HSI configuration");
    }

    if (mode == 1) {
        startHsiPolling();
    } else {
        stopHsiPolling();
    }
}

void VN110Bridge::startHsiPolling()
{
    hsi_has_previous_ = false;
    hsi_converged_count_ = 0;

    double interval = get_parameter("hsi_poll_interval").as_double();
    if (interval < 0.5) interval = 0.5;

    hsi_poll_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(interval * 1000)),
        std::bind(&VN110Bridge::pollHsiResults, this));

    RCLCPP_INFO(get_logger(), "HSI polling started (every %.1f s)", interval);
}

void VN110Bridge::stopHsiPolling()
{
    if (hsi_poll_timer_) {
        hsi_poll_timer_->cancel();
        hsi_poll_timer_.reset();
        hsi_has_previous_ = false;
        hsi_converged_count_ = 0;
        RCLCPP_INFO(get_logger(), "HSI polling stopped");
    }
}

void VN110Bridge::pollHsiResults()
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    EstMagCal est;
    if (sensor_.readRegister(&est) != VN::Error::None) {
        RCLCPP_WARN(get_logger(), "HSI poll: failed to read Register 47");
        return;
    }

    std::array<float, 12> current = {
        est.magGain00, est.magGain01, est.magGain02,
        est.magGain10, est.magGain11, est.magGain12,
        est.magGain20, est.magGain21, est.magGain22,
        est.magBiasX,  est.magBiasY,  est.magBiasZ
    };

    RCLCPP_INFO(get_logger(),
        "HSI Results (Reg47):\n"
        "  Gain: [%.6f, %.6f, %.6f]\n"
        "        [%.6f, %.6f, %.6f]\n"
        "        [%.6f, %.6f, %.6f]\n"
        "  Bias: [%.6f, %.6f, %.6f]",
        current[0], current[1], current[2],
        current[3], current[4], current[5],
        current[6], current[7], current[8],
        current[9], current[10], current[11]);

    if (hsi_has_previous_) {
        float max_change = 0.0f;
        int max_change_idx = 0;
        for (size_t i = 0; i < 12; ++i) {
            float diff = std::abs(current[i] - hsi_prev_[i]);
            if (diff > max_change) {
                max_change = diff;
                max_change_idx = static_cast<int>(i);
            }
        }

        float threshold = static_cast<float>(get_parameter("hsi_convergence_threshold").as_double());
        constexpr int CONVERGE_REQUIRED = 3;

        static const char* param_names[] = {
            "Gain00", "Gain01", "Gain02",
            "Gain10", "Gain11", "Gain12",
            "Gain20", "Gain21", "Gain22",
            "BiasX",  "BiasY",  "BiasZ"
        };

        if (max_change < threshold) {
            hsi_converged_count_++;
        } else {
            hsi_converged_count_ = 0;
        }

        RCLCPP_INFO(get_logger(),
            "HSI Convergence: max_change=%.8f (%s) | threshold=%.8f | stable_polls=%d/%d | %s",
            max_change, param_names[max_change_idx], threshold,
            hsi_converged_count_, CONVERGE_REQUIRED,
            (hsi_converged_count_ >= CONVERGE_REQUIRED) ? "CONVERGED" : "converging...");

        if (hsi_converged_count_ >= CONVERGE_REQUIRED) {
            RCLCPP_WARN(get_logger(),
                "========================================\n"
                "  HSI CONVERGED! (%d consecutive stable polls)\n"
                "  To apply: set hsi_mode=0, hsi_apply_compensation=true\n"
                "  Then call /vn110/write_settings to save to flash\n"
                "========================================",
                hsi_converged_count_);
        }
    } else {
        RCLCPP_INFO(get_logger(), "HSI Convergence: first poll (waiting for next to compute changes)");
    }

    hsi_prev_ = current;
    hsi_has_previous_ = true;
}

void VN110Bridge::applyInitialHeading(const ParamMap& changed)
{
    double heading = getParamDouble(changed, "initial_heading");
    if (std::isnan(heading)) return;

    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (sensor_.setInitialHeading(static_cast<float>(heading)) == VN::Error::None) {
        RCLCPP_INFO(get_logger(), "Initial heading set to %.1f deg", heading);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to set initial heading");
    }
}

void VN110Bridge::applyWorldModelConfig(const ParamMap& changed)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    bool enabled = getParamBool(changed, "world_model_enabled");
    double lat   = getParamDouble(changed, "world_model_latitude");
    double lon   = getParamDouble(changed, "world_model_longitude");
    double alt   = getParamDouble(changed, "world_model_altitude");

    RefModelConfig cfg;
    cfg.enableMagModel     = enabled ? RefModelConfig::EnableMagModel::Enabled
                                     : RefModelConfig::EnableMagModel::Disabled;
    cfg.enableGravityModel = enabled ? RefModelConfig::EnableGravityModel::Enabled
                                     : RefModelConfig::EnableGravityModel::Disabled;
    cfg.resv1              = 0;
    cfg.resv2              = 0;
    cfg.recalcThreshold    = 1000;
    cfg.year               = 2025.0f;
    cfg.latitude           = lat;
    cfg.longitude          = lon;
    cfg.altitude           = alt;

    if (sensor_.writeRegister(&cfg) == VN::Error::None) {
        RCLCPP_INFO(get_logger(), "World Model: %s, Pos=(%.4f, %.4f, %.1fm)",
            enabled ? "ENABLED" : "DISABLED", lat, lon, alt);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply world model configuration");
    }
}

void VN110Bridge::applyDeltaThetaVelConfig()
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    DeltaThetaVelConfig dtv;
    dtv.integrationFrame      = DeltaThetaVelConfig::IntegrationFrame::NED;
    dtv.gyroCompensation      = DeltaThetaVelConfig::GyroCompensation::Bias;
    dtv.accelCompensation     = DeltaThetaVelConfig::AccelCompensation::BiasAndGravity;
    dtv.earthRateCompensation = DeltaThetaVelConfig::EarthRateCompensation::RateAndCoriolis;

    if (sensor_.writeRegister(&dtv) == VN::Error::None) {
        RCLCPP_INFO(get_logger(),
            "DeltaThetaVel Config: Frame=NED, Gyro=Bias, Accel=BiasAndGravity, EarthRate=RateAndCoriolis");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply DeltaThetaVel configuration");
    }
}

void VN110Bridge::publishPx4Euler(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    float roll, pitch, yaw;
    float w    = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];

    float sinp = 2.0f * (w * y - z * x);
    roll       = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    pitch      = std::abs(sinp) >= 1.0f ? std::copysign(M_PI / 2.0f, sinp) : std::asin(sinp);
    yaw        = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

    geometry_msgs::msg::Vector3Stamped euler;
    euler.header.stamp    = now();
    euler.header.frame_id = "fmu";
    euler.vector.x        = roll * 180.0f / M_PI;
    euler.vector.y        = pitch * 180.0f / M_PI;
    euler.vector.z        = yaw * 180.0f / M_PI;
    px4_euler_pub_->publish(euler);
}

void VN110Bridge::processAirspeed(const px4_msgs::msg::AirspeedValidated::SharedPtr msg)
{
    float airspeed = msg->true_airspeed_m_s;
    if (std::isnan(airspeed)) return;

    latest_airspeed_.store(std::max(0.0f, airspeed), std::memory_order_relaxed);

    geometry_msgs::msg::Vector3Stamped aided_vel;
    aided_vel.header.stamp    = now();
    aided_vel.header.frame_id = frame_id_;
    aided_vel.vector.x        = airspeed;
    aided_vel_pub_->publish(aided_vel);

    std_msgs::msg::UInt8 thresh_status;

    if (!velocity_aiding_enabled_) {
        thresh_status.data = 2;
        aid_vel_thrsh_pub_->publish(thresh_status);
        return;
    }

    float airspeed_clamped = std::max(0.0f, airspeed);
    double threshold = get_parameter("velocity_aiding_threshold").as_double();

    if (airspeed_clamped < threshold) {
        thresh_status.data = 1;
        aid_vel_thrsh_pub_->publish(thresh_status);

        if (vel_aid_active_) {
            vel_aid_active_ = false;
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            writeVelAiding(false);
            RCLCPP_INFO(get_logger(), "Velocity aiding disabled (below threshold)");
        }
    } else {
        thresh_status.data = 0;
        aid_vel_thrsh_pub_->publish(thresh_status);

        geometry_msgs::msg::Vector3Stamped sent_vel;
        sent_vel.header.stamp    = now();
        sent_vel.header.frame_id = frame_id_;
        sent_vel.vector.x        = airspeed_clamped;
        aided_vel_sent_pub_->publish(sent_vel);

        if (!vel_aid_active_) {
            vel_aid_active_ = true;
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            writeVelAiding(true);
            RCLCPP_INFO(get_logger(), "Velocity aiding enabled (above threshold)");
        }

        std::lock_guard<std::mutex> lock(sensor_mutex_);
        VN::Registers::VelocityAiding::VelAidingMeas vel;
        vel.velocityX = airspeed_clamped;
        vel.velocityY = 0.0f;
        vel.velocityZ = 0.0f;

        VN::Error err = sensor_.writeRegister(&vel);
        if (err != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "VelAidingMeas NOT confirmed: error %d",
                        static_cast<int>(err));
        }
    }
}

int main(int argc,
         char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VN110Bridge>());
    rclcpp::shutdown();
    return 0;
}
