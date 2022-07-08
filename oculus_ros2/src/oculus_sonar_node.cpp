using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "oculus_sonar_node.hpp"

using SonarDriver = narval::oculus::SonarDriver;

OculusSonarNode::OculusSonarNode() : Node("oculus_sonar")
{
    if (!this->has_parameter("frame_id")) {
        this->declare_parameter<string>("frame_id", "sonar");
    }
    // if (!this->has_parameter("ping_topic")) {
    //     this->declare_parameter<string>("ping_topic", "ping");
    // }
    // if (!this->has_parameter("status_topic")) {
    //     this->declare_parameter<string>("status_topic", "ping");
    // }
    if (!this->has_parameter("frequency_mode")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(1).set__to_value(2).set__step(1);
        param_desc.name = "frequency_mode";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Sonar beam frequency mode.\n1: Low frequency (1.2MHz, wide aperture).\n2: High frequency (2.1Mhz, narrow aperture).";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("frequency_mode", 1, param_desc);
    }
    if (!this->has_parameter("ping_rate")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(5).set__step(1);
        param_desc.name = "ping_rate";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Frequency of ping fires.\n0: 10Hz max ping rate.\n1: 15Hz max ping rate.\n2: 40Hz max ping rate.\n3: 5Hz max ping rate.\n4: 2Hz max ping rate.\n5: Standby mode (no ping fire).";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("ping_rate", 0, param_desc);
    }
    if (!this->has_parameter("data_depth")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(1).set__step(1);
        param_desc.name = "data_depth";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Ping data encoding bit count.\n0: Ping data encoded on 8bits.\n1: Ping data encoded on 16bits.";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("data_depth", 0, param_desc);
    }
    if (!this->has_parameter("nbeams")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(1).set__step(1);
        param_desc.name = "nbeams";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Number of ping beams.\n0: Oculus outputs 256 beams.\n1: Oculus outputs 512 beams.";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("nbeams", 0, param_desc);
    }
    if (!this->has_parameter("send_gain")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.name = "send_gain";
        param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
        param_desc.description = "Send range gain with data.";
        this->declare_parameter<bool>("send_gain", false);
    }
    if (!this->has_parameter("gain_assist")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.name = "gain_assist";
        param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
        param_desc.description = "Enable auto gain.";
        this->declare_parameter<bool>("gain_assist", false, param_desc);
    }
    if (!this->has_parameter("range")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.3).set__to_value(40.0).set__step(0.1);
        param_desc.name = "nbeams";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Sonar range (in meters), min=0.3, max=40.0.";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("range", 3.0, param_desc);
    }
    if (!this->has_parameter("gamma_correction")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(255).set__step(1);
        param_desc.name = "gamma_correction";
        param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
        param_desc.description = "Gamma correction, min=0, max=255.";
        param_desc.integer_range = {range};
        this->declare_parameter<int>("gamma_correction", 127, param_desc);
    }
    if (!this->has_parameter("gain_percent")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.1).set__to_value(100.0).set__step(0.1);
        param_desc.name = "gain_percent";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Gain percentage (%), min=0.1, max=100.0.";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("gain_percent", 50.0, param_desc);
    }
    if (!this->has_parameter("sound_speed")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(1400.0).set__to_value(1600.0).set__step(0.1);
        param_desc.name = "sound_speed";
        param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        param_desc.description = "Sound speed (in m/s, set to 0 for it to be calculated using salinity), min=1400.0, max=1600.0.";
        param_desc.floating_point_range = {range};
        this->declare_parameter<double>("sound_speed", 0.0, param_desc);
    }
    if (!this->has_parameter("use_salinity")) {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.name = "use_salinity";
        param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
        param_desc.description = "Use salinity to calculate sound_speed.";
        this->declare_parameter<bool>("use_salinity", true, param_desc);
    }
    if (!this->has_parameter("salinity")) {
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
    
    this->ping_publisher_ = this->create_publisher<oculus_interfaces::msg::OculusStampedPing>(ping_topic_, 100);
    this->status_publisher_ = this->create_publisher<oculus_interfaces::msg::OculusStatus>(status_topic_, 100);

    this->sonar_driver_ = std::make_shared<SonarDriver>(this->io_service_.io_service());
    // this->sonar_driver_ = SonarDriver(this->io_service_.io_service());
    this->io_service_.start();
    if(!this->sonar_driver_->wait_next_message()) {
        std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
                  << "Is it properly connected ?" << std::endl;
    }
    this->sonar_driver_->add_status_callback(std::bind(&OculusSonarNode::publish_status, this, std::placeholders::_1));
    this->sonar_driver_->add_ping_callback(std::bind(&OculusSonarNode::publish_ping, this, std::placeholders::_1, std::placeholders::_2));
    // callback on dummy messages to reactivate the pings as needed
    this->sonar_driver_->add_dummy_callback(std::bind(&OculusSonarNode::handle_dummy, this));
}

OculusSonarNode::~OculusSonarNode()
{
    this->io_service_.stop();
}

void OculusSonarNode::publish_status(const OculusStatusMsg& status)
{
    static oculus_interfaces::msg::OculusStatus msg;
    
    narval::oculus::copy_to_ros(msg, status);

    this->status_publisher_->publish(msg);
}

inline rclcpp::Time to_ros_stamp(const SonarDriver::TimePoint& stamp)
{
    size_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(
        stamp.time_since_epoch()).count();
    size_t seconds = nano / 1000000000;
    return rclcpp::Time(seconds, nano - 1000000000*seconds);
}

void OculusSonarNode::publish_ping(const OculusSimplePingResult& pingMetadata,
                  const std::vector<uint8_t>& pingData)
{
    static oculus_interfaces::msg::OculusStampedPing msg;
     
    
    if(this->count_subscribers(this->ping_topic_) == 0) {
        cout << "Going to standby mode" << endl;
        this->sonar_driver_->standby();
        //return;
    }
    
    narval::oculus::copy_to_ros(msg.ping, pingMetadata);
    msg.ping.data.resize(pingData.size());
    for(int i = 0; i < msg.ping.data.size(); i++)
        msg.ping.data[i] = pingData[i];

    msg.header.stamp    = to_ros_stamp(this->sonar_driver_->last_header_stamp());
    msg.header.frame_id = "oculus_sonar";
    this->ping_publisher_->publish(msg);
}

void OculusSonarNode::handle_dummy()
{
    if(this->count_subscribers(this->ping_topic_) > 0) {
        cout << "Exiting standby mode" << endl;
        this->sonar_driver_->resume();
    }
}

rcl_interfaces::msg::SetParametersResult OculusSonarNode::set_config_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    SonarDriver::PingConfig currentConfig;
    std::memset(&currentConfig, 0, sizeof(currentConfig));
    // flags
    currentConfig.flags = 0x09; // always in meters, simple ping

    bool use_salinity;
    for (const rclcpp::Parameter & param : parameters) {
        // try {
        if (param.get_name() == "frequency_mode") {
            currentConfig.masterMode = param.as_int();
        } else if (param.get_name() == "ping_rate") {
            switch(param.as_int())
            {
                case 0: currentConfig.pingRate = pingRateNormal;  break;
                case 1: currentConfig.pingRate = pingRateHigh;    break;
                case 2: currentConfig.pingRate = pingRateHighest; break;
                case 3: currentConfig.pingRate = pingRateLow;     break;
                case 4: currentConfig.pingRate = pingRateLowest;  break;
                case 5: currentConfig.pingRate = pingRateStandby; break;
                default:break;
            }
        } else if (param.get_name() == "data_depth") {
            switch(param.as_int())
            {
                case 0: // 8 bits
                    break;
                case 1: // 16 bits
                    currentConfig.flags |= 0x02;
                    break;
                default:break;
            }
        } else if (param.get_name() == "nbeams") {
            switch(param.as_int())
            {
                case 0: // 256 beams
                    break;
                case 1: // 512 beams
                    currentConfig.flags |= 0x40;
                    break;
                default:break;
            }
        } else if (param.get_name() == "send_gain") {
            if(param.as_bool())
                currentConfig.flags |= 0x04;

        } else if (param.get_name() == "gain_assist") {
            if(param.as_bool())
                currentConfig.flags |= 0x10;

        } else if (param.get_name() == "range") {
            currentConfig.range = param.as_double();

        } else if (param.get_name() == "gamma_correction") {
            currentConfig.gammaCorrection = param.as_int();

        } else if (param.get_name() == "gain_percent") {
            currentConfig.gainPercent = param.as_double();

        } else if (param.get_name() == "use_salinity") {
            use_salinity = param.as_bool();
            if(use_salinity)
                currentConfig.speedOfSound = 0.0;

        } else if (param.get_name() == "sound_speed") {
            if(!use_salinity)
                currentConfig.speedOfSound = param.as_double();

        } else if (param.get_name() == "salinity") {
            currentConfig.salinity = param.as_double();
        }
        // } catch (const std::runtime_error & e) {
        // result.successful = false;
        // RCLCPP_WARN(get_node()->get_logger(), "%s", e.what());
        // }
    }
    // send config to Oculus sonar and wait for feedback
    auto feedback = this->sonar_driver_->request_ping_config(currentConfig);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    if(currentConfig.masterMode   != feedback.masterMode)
    {
        result.successful = false;
        result.reason.append("frequency_mode ");
    }
    //currentConfig.pingRate      != feedback.pingRate // is broken (?) sonar side
    if((currentConfig.flags & 0x02) ? 1 : 0 != (feedback.flags & 0x02) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("data_depth ");
    }
    if((currentConfig.flags & 0x04) ? 1 : 0 != (feedback.flags & 0x04) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("send_gain ");
    }
    if((currentConfig.flags & 0x10) ? 1 : 0 != (feedback.flags & 0x10) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("gain_assist ");
    }
    if((currentConfig.flags & 0x40) ? 1 : 0 != (feedback.flags & 0x40) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("nbeams ");
    }
    if(currentConfig.range            != feedback.range)
    {
        result.successful = false;
        result.reason.append("range ");
    }
    if(currentConfig.gammaCorrection != feedback.gammaCorrection)
    {
        result.successful = false;
        result.reason.append("gamma_correction ");
    }
    if(currentConfig.gainPercent     != feedback.gainPercent)
    {
        result.successful = false;
        result.reason.append("gain_percent ");
    }
    if(currentConfig.speedOfSound      != feedback.speedOfSound)
    {
        result.successful = false;
        result.reason.append("sound_speed ");
    }
    if(currentConfig.salinity         != feedback.salinity)
    {
        result.successful = false;
        result.reason.append("salinity ");
    }
    //(currentConfig.flags & 0x08) ? 1 : 0 != (feedback.flags & 0x08) ? 1 : 0
    
    if(!result.successful)
        result.reason.append("failed to be changed.");

    return result;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OculusSonarNode>());
    rclcpp::shutdown();
    return 0;
}