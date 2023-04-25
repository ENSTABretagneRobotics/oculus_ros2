
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "oculus_sonar_node.hpp"

// TODO(hugoyvrn)
#include "bitset"
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>

using SonarDriver = oculus::SonarDriver;

OculusSonarNode::OculusSonarNode() : Node("oculus_sonar"),
                                     sonar_driver_(std::make_shared<SonarDriver>(this->io_service_.io_service())),
                                     sonar_viewer(static_cast<rclcpp::Node *>(this)),
                                     frame_id((this->declare_parameter<std::string>("frame_id"), "sonar")),
                                     temperature_warn_limit(this->declare_parameter<double>("temperature_warn", 30.)),
                                     temperature_stop_limit(this->declare_parameter<double>("temperature_stop", 35.))
{
    RCLCPP_INFO_STREAM(get_logger(), "temperature_warn_limit = " << temperature_warn_limit);

    if (!this->has_parameter("standby"))
    {
        this->declare_parameter<bool>("standby", false);
    }
    if (!this->has_parameter("frequency_mode"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(1).set__to_value(2).set__step(1);
        param_desc.name = "frequency_mode";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Sonar beam frequency mode.\n\t1: Low frequency (1.2MHz, wide aperture).\n\t2: High frequency (2.1Mhz, narrow aperture).";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("frequency_mode", 1, param_desc);
    }
    if (!this->has_parameter("ping_rate"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(5).set__step(1);
        param_desc.name = "ping_rate";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Frequency of ping fires.\n\t0: 10Hz max ping rate.\n\t1: 15Hz max ping rate.\n\t2: 40Hz max ping rate.\n\t3: 5Hz max ping rate.\n\t4: 2Hz max ping rate.\n\t5: Standby mode (no ping fire).";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("ping_rate", 0, param_desc);
    }
    if (!this->has_parameter("data_depth"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(1).set__step(1);
        param_desc.name = "data_depth";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Ping data encoding bit count.\n\t0: Ping data encoded on 8bits.\n\t1: Ping data encoded on 16bits.";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("data_depth", 0, param_desc);
    }
    if (!this->has_parameter("nbeams"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;

        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(1).set__step(1);
        param_desc.name = "nbeams";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Number of ping beams.\n\t0: Oculus outputs 256 beams.\n\t1: Oculus outputs 512 beams.";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("nbeams", 0, param_desc);
    }
    if (!this->has_parameter("gain_assist"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.name = "gain_assist";
        param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
        param_desc.description = "Enable auto gain.";
        this->declare_parameter<bool>("gain_assist", false, param_desc);
    }
    if (!this->has_parameter("range"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.3).set__to_value(40.0).set__step(0.1);
        param_desc.name = "nbeams";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Sonar range (in meters), min=0.3, max=40.0.";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("range", 40., param_desc);
    }
    if (!this->has_parameter("gamma_correction"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(255).set__step(1);
        param_desc.name = "gamma_correction";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Gamma correction, min=0, max=255.";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("gamma_correction", 127, param_desc);
    }
    if (!this->has_parameter("gain_percent"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.1).set__to_value(100.0).set__step(0.1);
        param_desc.name = "gain_percent";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Gain percentage (%), min=0.1, max=100.0.";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("gain_percent", 50.0, param_desc);
    }
    if (!this->has_parameter("sound_speed"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.0).set__to_value(1600.0).set__step(0.1); // min = 1400.0 but must include 0.0 for configuration
        param_desc.name = "sound_speed";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Sound speed (in m/s, set to 0 for it to be calculated using salinity), min=1400.0, max=1600.0.";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("sound_speed", 0.0, param_desc);
    }
    if (!this->has_parameter("use_salinity"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.name = "use_salinity";
        param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
        param_desc.description = "Use salinity to calculate sound_speed.";
        this->declare_parameter<bool>("use_salinity", true, param_desc);
    }
    if (!this->has_parameter("salinity"))
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.0).set__to_value(100.0).set__step(0.1);
        param_desc.name = "salinity";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Salinity (in parts per thousand (ppt,ppm,g/kg), used to calculate sound speed if needed), min=0.0, max=100";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("salinity", 0.0, param_desc);
    }

    this->param_cb_ = this->add_on_set_parameters_callback(std::bind(&OculusSonarNode::set_config_callback, this, std::placeholders::_1));

    this->status_publisher_ = this->create_publisher<oculus_interfaces::msg::OculusStatus>(get_parameter_or("status_topic", std::string("zzzz")), 100);
    this->ping_publisher_ = this->create_publisher<oculus_interfaces::msg::Ping>(get_parameter_or("ping_topic", std::string("eeee")), 100);
    this->temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(get_parameter_or("temperature_topic", std::string("rrrr")), 100); // TODO(hugoyvrn, size of the queue?)
    this->pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(get_parameter_or("pressure_topic", std::string("tttt")), 100);     // TODO(hugoyvrn, size of the queue?)

    // image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("fan_image", 10);

    this->io_service_.start();
    if (!this->sonar_driver_->wait_next_message())
    {
        std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
                  << "Is it properly connected ?" << std::endl;
    }
    // const oculus::PingMessage::ConstPtr ping;
    // sonar_viewer.publish_fan(ping);

    while (!this->sonar_driver_->connected()) // wait the sonar to connected
    {
        // std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
        //           << "Is it properly connected ?" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    update_parameters(currentSonarParameters, this->sonar_driver_->current_ping_config());
    for (const std::string &param_name : dynamic_parameters_names)
        set_config_callback(this->get_parameters(std::vector{param_name}));

    this->sonar_driver_->add_status_callback(std::bind(&OculusSonarNode::publish_status, this, std::placeholders::_1));
    this->sonar_driver_->add_ping_callback(std::bind(&OculusSonarNode::publish_ping, this, std::placeholders::_1));
    // callback on dummy messages to reactivate the pings as needed
    this->sonar_driver_->add_dummy_callback(std::bind(&OculusSonarNode::handle_dummy, this));
}

OculusSonarNode::~OculusSonarNode()
{
    this->io_service_.stop();
}

void OculusSonarNode::publish_status(const OculusStatusMsg &status) const
{

    if (status.partNumber != 1042) // TODO(handle other versions).
        RCLCPP_ERROR_STREAM(get_logger(), "The sonar version seems to be different than M1200d. This driver is not suppose to work with your sonar.");

    static oculus_interfaces::msg::OculusStatus msg;
    oculus::copy_to_ros(msg, status);
    this->status_publisher_->publish(msg);
}

inline rclcpp::Time to_ros_stamp(const SonarDriver::TimePoint &stamp)
{
    size_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      stamp.time_since_epoch())
                      .count();
    size_t seconds = nano / 1000000000;
    return rclcpp::Time(seconds, nano - 1000000000 * seconds);
}

void OculusSonarNode::update_ros_config()
{
    std::shared_lock l(param_mutex);

    update_ros_config_for_param<int>(currentRosParameters.frequency_mode, currentSonarParameters.frequency_mode, "frequency_mode", "master_mode");
    update_ros_config_for_param<double>(currentRosParameters.range, currentSonarParameters.range, "range");
    update_ros_config_for_param<double>(currentRosParameters.gain_percent, currentSonarParameters.gain_percent, "gain_percent");
    update_ros_config_for_param<double>(currentRosParameters.sound_speed, currentSonarParameters.sound_speed, "sound_speed", "speed_of_sound_used");
    update_ros_config_for_param<int>(currentRosParameters.ping_rate, currentSonarParameters.ping_rate, "ping_rate");
    update_ros_config_for_param<int>(currentRosParameters.data_depth, currentSonarParameters.data_depth, "data_depth");
    update_ros_config_for_param<bool>(currentRosParameters.gain_assist, currentSonarParameters.gain_assist, "gain_assist");
    update_ros_config_for_param<int>(currentRosParameters.gamma_correction, currentSonarParameters.gamma_correction, "gamma_correction");
    update_ros_config_for_param<bool>(currentRosParameters.use_salinity, currentSonarParameters.use_salinity, "use_salinity");
    update_ros_config_for_param<double>(currentRosParameters.salinity, currentSonarParameters.salinity, "salinity");
}

void OculusSonarNode::publish_ping(const oculus::PingMessage::ConstPtr &ping)
{
    if (this->ping_publisher_->get_subscription_count() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "There is no subscriber to the ping topic.");
        RCLCPP_INFO(this->get_logger(), "Going to standby mode");
        this->sonar_driver_->standby();
    }
    if (currentSonarParameters.ping_rate == 5)
    {
        RCLCPP_INFO(this->get_logger(), "ping_rate mode is seted to 5.");
        RCLCPP_INFO(this->get_logger(), "Going to standby mode");
        this->sonar_driver_->standby();
    }

    if (ping->temperature() >= temperature_stop_limit)
    {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Temperature of sonar is to high (" << ping->temperature() << "°C). Make sur the sonar is underwatter. Security limit set at " << temperature_stop_limit << "°C");
        is_in_standby_mode = true;
        this->set_parameter(rclcpp::Parameter("standby", true));
    }
    else if (ping->temperature() >= temperature_warn_limit)
        RCLCPP_WARN_STREAM(this->get_logger(), "Temperature of sonar is to high (" << ping->temperature() << "°C). Make sur the sonar is underwatter. Security limit set at " << temperature_stop_limit << "°C");

    if (is_in_standby_mode)
    {
        RCLCPP_INFO(this->get_logger(), "Going to standby mode");
        this->sonar_driver_->standby();
        return;
    }

    // Update current config with ping informations
    currentSonarParameters.frequency_mode = ping->master_mode();
    currentSonarParameters.range = ping->range();
    currentSonarParameters.gain_percent = ping->gain_percent();
    currentSonarParameters.sound_speed = ping->speed_of_sound_used();

    static oculus_interfaces::msg::Ping msg;
    msg.header.frame_id = frame_id;
    oculus::copy_to_ros(msg, ping);
    this->ping_publisher_->publish(msg);

    sensor_msgs::msg::Temperature temperature_ros_msg;
    temperature_ros_msg.header = msg.header;
    temperature_ros_msg.temperature = msg.temperature; // Measurement of the Temperature in Degrees Celsius
    temperature_ros_msg.variance = 0;                  // 0 is interpreted as variance unknown
    this->temperature_publisher_->publish(temperature_ros_msg);

    sensor_msgs::msg::FluidPressure pressure_ros_msg;
    pressure_ros_msg.header = msg.header;
    pressure_ros_msg.fluid_pressure = msg.pressure; // Absolute pressure reading in Pascals.
    pressure_ros_msg.variance = 0;                  // 0 is interpreted as variance unknown
    this->pressure_publisher_->publish(pressure_ros_msg);

    // this->image_publisher_->publish(sonar_viewer.publish_fan(ping));
    // sonar_viewer.publish_fan(ping);

    update_ros_config();
}

void OculusSonarNode::handle_dummy() const
{
    if (!is_in_standby_mode && this->ping_publisher_->get_subscription_count() > 0 && currentSonarParameters.ping_rate != 5)
    {
        RCLCPP_INFO(this->get_logger(), "Exiting standby mode");
        this->sonar_driver_->resume();
    }
}

void OculusSonarNode::update_parameters(rosParameters &parameters, const std::vector<rclcpp::Parameter> &new_parameters)
{
    for (const rclcpp::Parameter &new_param : new_parameters)
    {
        if (new_param.get_name() == "frequency_mode")
            parameters.frequency_mode = new_param.as_int();
        else if (new_param.get_name() == "ping_rate")
            parameters.ping_rate = new_param.as_int();
        else if (new_param.get_name() == "data_depth")
            parameters.data_depth = new_param.as_int();
        else if (new_param.get_name() == "nbeams")
            parameters.nbeams = new_param.as_int();
        else if (new_param.get_name() == "gain_assist")
            parameters.gain_assist = new_param.as_bool();
        else if (new_param.get_name() == "range")
            parameters.range = new_param.as_double();
        else if (new_param.get_name() == "gamma_correction")
            parameters.gamma_correction = new_param.as_int();
        else if (new_param.get_name() == "gain_percent")
            parameters.gain_percent = new_param.as_double();
        else if (new_param.get_name() == "sound_speed")
            parameters.sound_speed = new_param.as_double();
        else if (new_param.get_name() == "use_salinity")
            parameters.use_salinity = new_param.as_bool();
        else if (new_param.get_name() == "salinity")
            parameters.salinity = new_param.as_double();
        else if (!(new_param.get_name() == "standby"))
            RCLCPP_WARN_STREAM(get_logger(), "Wrong parameter to set : new_param = " << new_param << ". Not seted");
    }
    // RCLCPP_INFO_STREAM(get_logger(), "new_parameters = " << new_parameters);
}

void OculusSonarNode::update_parameters(rosParameters &parameters, SonarDriver::PingConfig feedback)
{
    std::vector<rclcpp::Parameter> new_parameters;
    // OculusMessageHeader head;     // The standard message header
    // uint16_t oculusId;         // Fixed ID 0x4f53
    // uint16_t srcDeviceId;      // The device id of the source
    // uint16_t dstDeviceId;      // The device id of the destination
    // uint16_t msgId;            // Message identifier
    // uint16_t msgVersion;
    // uint32_t payloadSize;      // The size of the message payload (header not included)
    // uint16_t spare2;
    // uint8_t masterMode;           // mode 0 is flexi mode, needs full fire message (not available for third party developers)
    //                               // mode 1 - Low Frequency Mode (wide aperture, navigation)
    //                               // mode 2 - High Frequency Mode (narrow aperture, target identification)
    // uint8_t pingRate;             // Sets the maximum ping rate. was PingRateType
    // uint8_t networkSpeed;         // Used to reduce the network comms speed (useful for high latency shared links)
    // uint8_t gammaCorrection;      // 0 and 0xff = gamma correction = 1.0
    //                               // Set to 127 for gamma correction = 0.5
    // uint8_t flags;                // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
    //                               // bit 1: 0 = 8 bit data, 1 = 16 bit data // inverted ?
    //                               // bit 2: 0 = wont send gain, 1 = send gain
    //                               // bit 3: 0 = send full return message, 1 = send simple return message
    //                               // bit 4: gain assist ?
    //                               // bit 5: ?
    //                               // bit 6: enable 512 beams
    //                               // bit 7: ?
    // double range;                 // The range demand in percent or m depending on flags
    // double gainPercent;           // The gain demand
    // double speedOfSound;          // ms-1, if set to zero then internal calc will apply using salinity
    // double salinity;              // ppt, set to zero if we are in fresh water

    // Checks
    if (!(feedback.flags & 0x01))
        RCLCPP_ERROR(get_logger(), "Range is attepreted as percent while ros driver assume range is interpreted as meters.");
    if (!(feedback.flags & 0x04))
        RCLCPP_ERROR(get_logger(), "The sonar don't send gain while ros driver assume gains are sended. Data is uncomplete.");
    if (!(feedback.flags & 0x08))
        RCLCPP_ERROR(get_logger(), "The sonar don't use simple ping message while ros driver assume simple ping are used.");
    {
        // TODO(hugoyvrn)
        // feedback.flags & 0x20  // What for ?
    } {
        // TODO(hugoyvrn)
        // feedback.flags & 0x40  // What for ?
    }

    new_parameters.push_back(rclcpp::Parameter("frequency_mode", feedback.masterMode)); // "frequency_mode"
    new_parameters.push_back(rclcpp::Parameter("ping_rate", feedback.pingRate));
    new_parameters.push_back(rclcpp::Parameter("data_depth", int(feedback.flags & 0x02)));   // data_depth
    new_parameters.push_back(rclcpp::Parameter("nbeams", int(feedback.flags & 0x30)));       // nbeams
    new_parameters.push_back(rclcpp::Parameter("gain_assist", bool(feedback.flags & 0x10))); // gain_assist
    new_parameters.push_back(rclcpp::Parameter("range", feedback.range));
    new_parameters.push_back(rclcpp::Parameter("gamma_correction", feedback.gammaCorrection));
    new_parameters.push_back(rclcpp::Parameter("gain_percent", feedback.gainPercent));
    new_parameters.push_back(rclcpp::Parameter("sound_speed", feedback.speedOfSound));
    // // use_salinity // TODO(hugoyvrn)
    // {
    //     rclcpp::Parameter param("use_salinity", );
    //     new_parameters.push_back(param);
    // }

    new_parameters.push_back(rclcpp::Parameter("salinity", feedback.salinity));
    update_parameters(parameters, new_parameters);
}

void OculusSonarNode::send_param_to_sonar(rclcpp::Parameter param, rcl_interfaces::msg::SetParametersResult result)
{
    SonarDriver::PingConfig newConfig = currentConfig; // To avoid to creat a new SonarDriver::PingConfig from ros parameters

    if (param.get_name() == "frequency_mode")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating frequency_mode to " << param.as_int() << " (1: 1.2MHz, 2: 2MHz).");
        newConfig.masterMode = param.as_int();
    }
    else if (param.get_name() == "ping_rate")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating ping_rate to " << param.as_int() << " (0: 10Hz, 1: 15Hz, 2: 40Hz, 3: 5Hz, 4: 2Hz, 5: Standby mode).");
        switch (param.as_int())
        {
        case 0:
            newConfig.pingRate = pingRateNormal;
            break; // 10Hz
        case 1:
            newConfig.pingRate = pingRateHigh;
            break; // 15Hz
        case 2:
            newConfig.pingRate = pingRateHighest;
            break; // 40Hz
        case 3:
            newConfig.pingRate = pingRateLow;
            break; // 5Hz
        case 4:
            newConfig.pingRate = pingRateLowest;
            break; // 2Hz
        case 5:
            newConfig.pingRate = pingRateStandby;
            break; // standby mode
        default:
            break;
        }
    }
    else if (param.get_name() == "data_depth")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating data_depth to " << param.as_int() << " (0: 8 bits, 1: 16 bits).");
        switch (param.as_int())
        {
        case 0: // 8 bits
            newConfig.flags &= ~0x02;
            break;
        case 1: // 16 bits
            newConfig.flags |= 0x02;
            break;
        default:
            break;
        }
    }
    else if (param.get_name() == "nbeams")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating nbeams to " << param.as_int() << " (0: 256 beams, 1: 512 beams).");
        switch (param.as_int())
        {
        case 0: // 256 beams
            newConfig.flags &= ~0x40;
            break;
        case 1: // 512 beams
            newConfig.flags |= 0x40;
            break;
        default:
            break;
        }
    }
    else if (param.get_name() == "gain_assist")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating gain_assist to " << param.as_bool());
        if (param.as_bool())
            newConfig.flags |= 0x10;
        else
            newConfig.flags &= ~0x10;
    }
    else if (param.get_name() == "range")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating range to " << param.as_double() << "m.");
        newConfig.range = param.as_double();
    }
    else if (param.get_name() == "gamma_correction")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating gamma_correction to " << param.as_int());
        newConfig.gammaCorrection = param.as_int();
    }
    else if (param.get_name() == "gain_percent")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating gain_percent to " << param.as_double() << "%.");
        newConfig.gainPercent = param.as_double();
    }
    else if (param.get_name() == "use_salinity")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating use_salinity to " << param.as_bool());
        if (param.as_bool())
            newConfig.speedOfSound = 0.0;
    }
    else if (param.get_name() == "sound_speed")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating sound_speed to " << param.as_double() << " m/s.");
        if (!param.as_bool())
        {
            if (param.as_double() >= 1400.0 && param.as_double() <= 1600.0)
                newConfig.speedOfSound = param.as_double();
            else
                RCLCPP_INFO_STREAM(this->get_logger(), "Speed of sound must be between 1400.0 and 1600.0.");
        }
    }
    else if (param.get_name() == "salinity")
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Updating salinity to " << param.as_double() << " parts per thousand (ppt,ppm,g/kg).");
        newConfig.salinity = param.as_double();
    }

    newConfig.flags |= 0x01    // always in meters
                       | 0x04  // force send gain to true
                       | 0x08; // use simple ping

    // send config to Oculus sonar and wait for feedback
    SonarDriver::PingConfig feedback = this->sonar_driver_->request_ping_config(newConfig);
    currentConfig = feedback;
    update_parameters(currentSonarParameters, feedback);

    if (feedback.flags != newConfig.flags)
    {
        RCLCPP_INFO_STREAM(get_logger(), "newConfig.flags = " << std::bitset<8>(newConfig.flags));
        RCLCPP_INFO_STREAM(get_logger(), "feedback.flags = " << std::bitset<8>(feedback.flags));
    }
    else
        RCLCPP_INFO_STREAM(get_logger(), "feedback.flags = " << std::bitset<8>(feedback.flags)); // TODO(hugoyrn, to remove)

    // Warning
    if (!(feedback.flags & 0x04))
        RCLCPP_ERROR(this->get_logger(), "The oculus do not send gains. There is an error. Data is not complete.");

    handle_feedback_for_param<double>(result, param, newConfig.masterMode, feedback.masterMode, "masterMode", "frequency_mode");
    // newConfig.pingRate      != feedback.pingRate // is broken (?) sonar side TODO
    handle_feedback_for_param<int>(result, param, (newConfig.flags & 0x02) ? 1 : 0, (feedback.flags & 0x02) ? 1 : 0,
                                   "data_depth");
    handle_feedback_for_param<bool>(result, param, (newConfig.flags & 0x10) ? 1 : 0, (feedback.flags & 0x10) ? 1 : 0,
                                    "gain_assist");
    handle_feedback_for_param<int>(result, param, (newConfig.flags & 0x40) ? 1 : 0, (feedback.flags & 0x40) ? 1 : 0,
                                   "nbeams");
    handle_feedback_for_param<double>(result, param, newConfig.range, feedback.range, "range");
    handle_feedback_for_param<int>(result, param, newConfig.gammaCorrection, feedback.gammaCorrection, "gamma_correction");
    handle_feedback_for_param<double>(result, param, newConfig.gainPercent, feedback.gainPercent, "gain_percent");
    handle_feedback_for_param<double>(result, param, newConfig.speedOfSound, feedback.speedOfSound, "sound_speed"); // TODO(hugoyvrn)
    handle_feedback_for_param<double>(result, param, newConfig.salinity, feedback.salinity, "salinity");
}

rcl_interfaces::msg::SetParametersResult OculusSonarNode::set_config_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    std::shared_lock l(param_mutex);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    // TODO(hugoyvrn, 1 param par 1 param)
    if (parameters.size() != 1)
    {
        RCLCPP_WARN(get_logger(), "You should set parmeters one by one.");
        RCLCPP_INFO_STREAM(get_logger(), "parameters = " << parameters);
    }

    for (const rclcpp::Parameter &param : parameters)
    {
        if (param.get_name() == "standby")
            is_in_standby_mode = param.as_bool();
        else if (std::find(dynamic_parameters_names.begin(), dynamic_parameters_names.end(), param.get_name()) != dynamic_parameters_names.end())
            send_param_to_sonar(param, result);
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "Wrong dynamic parameter to set : param = " << param << ". Not seted");
            result.successful = true; // TODO(hugoyvrn, true or false ?)
            result.reason = "TODO(hugoyvrn)";
        }
    }

    if (result.successful) // If the parameters will be updated to ros
        update_parameters(currentRosParameters, parameters);

    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OculusSonarNode>());
    rclcpp::shutdown();
    return 0;
}
