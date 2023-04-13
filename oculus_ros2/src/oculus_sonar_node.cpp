using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "oculus_sonar_node.hpp"

// TODO(hugoyvrn)
#include "bitset"
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>

using SonarDriver = oculus::SonarDriver;

OculusSonarNode::OculusSonarNode() : Node("oculus_sonar")
{

    if (!this->has_parameter("frame_id"))
    {
        this->declare_parameter<string>("frame_id", "sonar");
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
    this->get_parameter("ping_topic", ping_topic_);
    this->get_parameter("status_topic", status_topic_);

    this->param_cb_ = this->add_on_set_parameters_callback(std::bind(&OculusSonarNode::set_config_callback, this, std::placeholders::_1));

    this->ping_publisher_ = this->create_publisher<oculus_interfaces::msg::Ping>(ping_topic_, 100);
    this->status_publisher_ = this->create_publisher<oculus_interfaces::msg::OculusStatus>(status_topic_, 100);

    this->sonar_driver_ = std::make_shared<SonarDriver>(this->io_service_.io_service());
    this->io_service_.start();
    if (!this->sonar_driver_->wait_next_message())
    {
        std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
                  << "Is it properly connected ?" << std::endl;
    }
    while (!this->sonar_driver_->connected()) // TODO(hugoyvrn, Not working if the sonar is not connected at startup)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    for (const std::string &param_name : parameters_names)
    {
        currentConfig = this->sonar_driver_->current_ping_config();
        set_config_callback(this->get_parameters(std::vector{param_name}));
    }

    this->sonar_driver_->add_status_callback(std::bind(&OculusSonarNode::publish_status, this, std::placeholders::_1));
    this->sonar_driver_->add_ping_callback(std::bind(&OculusSonarNode::publish_ping, this, std::placeholders::_1));
    // callback on dummy messages to reactivate the pings as needed
    this->sonar_driver_->add_dummy_callback(std::bind(&OculusSonarNode::handle_dummy, this));
}

OculusSonarNode::~OculusSonarNode()
{
    this->io_service_.stop();
}

void OculusSonarNode::publish_status(const OculusStatusMsg &status)
{
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

void OculusSonarNode::update_ros_param_from_ping_msg(auto &currentConfig_param, const auto &msg_param, const std::string &ros_param_name, const std::string &param_name)
{
    if (currentConfig_param != msg_param)
    {
        this->remove_on_set_parameters_callback(this->param_cb_.get());
        RCLCPP_INFO_STREAM(this->get_logger(), "coucou " << param_name << " : ros = " << currentConfig_param << " topic = " << msg_param);
        currentConfig_param = msg_param;
        this->set_parameter(rclcpp::Parameter(ros_param_name, msg_param));
        this->param_cb_ = this->add_on_set_parameters_callback(std::bind(&OculusSonarNode::set_config_callback, this, std::placeholders::_1));
    }
}

void OculusSonarNode::update_ros_config_from_ping_msg(const oculus_interfaces::msg::Ping &msg)
{
    std::shared_lock l(param_mutex);

    update_ros_param_from_ping_msg(currentConfig.masterMode, msg.master_mode, "frequency_mode", "master_mode");
    update_ros_param_from_ping_msg(currentConfig.range, msg.range, "range", "range");
    update_ros_param_from_ping_msg(currentConfig.gainPercent, msg.gain_percent, "gain_percent", "gain_percent");
    update_ros_param_from_ping_msg(currentConfig.speedOfSound, msg.speed_of_sound_used, "sound_speed", "speed_of_sound_used");
    // update_ros_param_from_ping_msg(currentConfig.pingRate, msg.ping_rate, "ping_rate", "ping_rate");
    // update_ros_param_from_ping_msg(currentConfig.dataDepth, msg.data_depth, "data_depth", "data_depth");
    // update_ros_param_from_ping_msg(currentConfig.gainAssist, msg.gain_assist, "gain_assist", "gain_assist");
    // update_ros_param_from_ping_msg(currentConfig.gammaCorrection, msg.gamma_correction, "gamma_correction", "gamma_correction");
    // update_ros_param_from_ping_msg(currentConfig.use_salinity, msg.use_salinity, "use_salinity", "use_salinity");
    // update_ros_param_from_ping_msg(currentConfig.salinity, msg.salinity, "salinity", "salinity");
}


void OculusSonarNode::publish_ping(const oculus::PingMessage::ConstPtr &ping)
{
    static oculus_interfaces::msg::Ping msg;

    if (this->ping_publisher_->get_subscription_count() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Going to standby mode");
        this->sonar_driver_->standby();
        return;
    }

    oculus::copy_to_ros(msg, ping);
    update_ros_config_from_ping_msg(msg);
    if (msg.temperature >= temperature_stop_limit)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Temperature of sonar is to high (" << msg.temperature << "°C). Make sur the sonar is underwatter. Security limit set at " << temperature_stop_limit << "°C");
        this->sonar_driver_->close_connection();
        RCLCPP_WARN(this->get_logger(), "Waiting 30s the sonar to cold down."); // TODO(hugoyvrn)
        std::this_thread::sleep_for(std::chrono::milliseconds(30000));
    }
    else if (msg.temperature >= temperature_warn_limit)
        RCLCPP_WARN_STREAM(this->get_logger(), "Temperature of sonar is to high (" << msg.temperature << "°C). Make sur the sonar is underwatter. Security limit set at " << temperature_stop_limit << "°C");
    this->ping_publisher_->publish(msg);
}

void OculusSonarNode::handle_dummy()
{
    if (this->ping_publisher_->get_subscription_count() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Exiting standby mode");
        this->sonar_driver_->resume();
    }
}

void OculusSonarNode::handle_feedback_for_param(rcl_interfaces::msg::SetParametersResult &result, const rclcpp::Parameter &param, const auto &old_val, const auto &new_val, const std::string &param_name, const std::string &param_name_to_display) const
{

    if (old_val != new_val)
    {
        std::string param_name_to_display_ = param_name_to_display == "" ? param_name : param_name_to_display;
        if (param.get_name() == param_name)
        {
            // if (parameters.size() == 1)
            result.successful = false;
            RCLCPP_WARN_STREAM(this->get_logger(), "Could not update " << param_name_to_display_);
            result.reason.append("Could not update " + param_name_to_display_ + ".\n");
        }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(), param_name_to_display_ << " change from " << old_val << " to " << new_val << " when updating the parameter " << param.get_name());
            result.reason.append(param_name_to_display_ + " change.\n");
        }
    }
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

    SonarDriver::PingConfig newConfig = currentConfig;
    // flags
    newConfig.flags = 0x01    // always in meters
                      | 0x04  // force send gain to true
                      | 0x08; // use simple ping

    bool use_salinity;
    for (const rclcpp::Parameter &param : parameters)
    {
        if (param.get_name() == "frequency_mode")
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Updating frequency_mode to " << param.as_int() << " (1: 1.2MHz, 2: 2.1MHz).");
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
            use_salinity = param.as_bool();
            if (use_salinity)
                newConfig.speedOfSound = 0.0;
        }
        else if (param.get_name() == "sound_speed")
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Updating sound_speed to " << param.as_double() << " m/s.");
            if (!use_salinity)
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

        // send config to Oculus sonar and wait for feedback
        SonarDriver::PingConfig feedback = this->sonar_driver_->request_ping_config(newConfig);
        if (feedback.flags != newConfig.flags)
        {
            RCLCPP_INFO_STREAM(get_logger(), "newConfig.flags = " << std::bitset<8>(newConfig.flags));
            RCLCPP_INFO_STREAM(get_logger(), "feedback.flags = " << std::bitset<8>(feedback.flags));
        }
        currentConfig = feedback;

        // Advertissements
        if (feedback.flags & 0x10)
        {
            RCLCPP_WARN(this->get_logger(), "gain_assit parameter is inable. To record data make sur to desable it. \n\tros2 set /oculus_sonar /gain_assist False");
        }
        if (!(feedback.flags & 0x04)) // TODO(hugoyvrn, to move elsewhere)
        {
            RCLCPP_WARN(this->get_logger(), "The oculus do not send gains. There is an error. The data is not complete.");
        }

        handle_feedback_for_param(result, param, newConfig.masterMode, feedback.masterMode, "masterMode", "frequency_mode");
        // newConfig.pingRate      != feedback.pingRate // is broken (?) sonar side TODO
        handle_feedback_for_param(result, param, (newConfig.flags & 0x02) ? 1 : 0, (feedback.flags & 0x02) ? 1 : 0,
                                  "data_depth");
        handle_feedback_for_param(result, param, (newConfig.flags & 0x10) ? 1 : 0, (feedback.flags & 0x10) ? 1 : 0,
                                  "gain_assist");
        handle_feedback_for_param(result, param, (newConfig.flags & 0x40) ? 1 : 0, (feedback.flags & 0x40) ? 1 : 0,
                                  "nbeams");
        handle_feedback_for_param(result, param, newConfig.range, feedback.range, "range");
        handle_feedback_for_param(result, param, newConfig.gammaCorrection, feedback.gammaCorrection, "gamma_correction");
        handle_feedback_for_param(result, param, newConfig.gainPercent, feedback.gainPercent, "gain_percent");
        handle_feedback_for_param(result, param, newConfig.speedOfSound, feedback.speedOfSound, "sound_speed"); // TODO(hugoyvrn)
        handle_feedback_for_param(result, param, newConfig.salinity, feedback.salinity, "salinity");
    }
    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OculusSonarNode>());
    rclcpp::shutdown();
    return 0;
}
