using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <include/conversions.h>
#include <include/oculus_sonar_node.hpp>

using SonarDriver = narval::oculus::SonarDriver;

OculusSonarNode::OculusSonarNode() : Node('oculus_sonar')
{
    if (!this->has_parameter("frame_id")) {
        this->declare_parameter<string>("frame_id", "sonar");
    }
    if (!this->has_parameter("ping_topic")) {
        this->declare_parameter<string>("ping_topic", "ping");
    }
    if (!this->has_parameter("status_topic")) {
        this->declare_parameter<string>("status_topic", "ping");
    }
    if (!this->has_parameter("frequency_mode")) {
        this->declare_parameter<int>("frequency_mode", 1);
    }
    if (!this->has_parameter("ping_rate")) {
        this->declare_parameter<int>("ping_rate", 0);
    }
    if (!this->has_parameter("data_depth")) {
        this->declare_parameter<int>("data_depth", 0);
    }
    if (!this->has_parameter("nbeams")) {
        this->declare_parameter<int>("nbeams", 0);
    }
    if (!this->has_parameter("send_gain")) {
        this->declare_parameter<bool>("send_gain", false);
    }
    if (!this->has_parameter("gain_assist")) {
        this->declare_parameter<bool>("gain_assist", false);
    }
    if (!this->has_parameter("range")) {
        this->declare_parameter<double>("range", 3.0);
    }
    if (!this->has_parameter("gamma_correction")) {
        this->declare_parameter<int>("gamma_correction", 127);
    }
    if (!this->has_parameter("gain_percent")) {
        this->declare_parameter<double>("gain_percent", 50.0);
    }
    if (!this->has_parameter("sound_speed")) {
        this->declare_parameter<double>("sound_speed", 0.0);
    }
    if (!this->has_parameter("use_salinity")) {
        this->declare_parameter<bool>("use_salinity", true);
    }
    if (!this->has_parameter("salinity")) {
        this->declare_parameter<double>("salinity", 0.0);
    }
    std::string pingTopic, statusTopic;
    this->get_parameter("ping_topic", pingTopic);
    this->get_parameter("status_topic", statusTopic);

    this->param_cb_ = this->add_on_set_parameters_callback(std::bind(&OculusSonarNode::set_config_callback, this, _1));
    
    this->ping_publisher_ = this->create_publisher<oculus_sonar::msg::OculusStampedPing>(pingTopic, 100);
    this->status_publisher_ = this->create_publisher<oculus_sonar::msg::OculusStatus>(statusTopic, 100);

    this->sonar_driver_ = SonarDriver(this->io_service_->io_service());
    this->io_service_->start();
    if(!this->sonar_driver_.wait_next_message()) {
        std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
                  << "Is it properly connected ?" << std::endl;
    }
    this->sonar_driver_->add_status_callback(&publish_status, this->status_publisher_);
    this->sonar_driver_->add_ping_callback(&publish_ping, this->sonar_driver_, this->ping_publisher_);
    // callback on dummy messages to reactivate the pings as needed
    this->sonar_driver_->add_dummy_callback(&handle_dummy, this->sonar_driver_, this->ping_publisher_);
}

OculusSonarNode::~OculusSonarNode()
{
    this->io_service_->stop();
}

rcl_interfaces::msg::SetParametersResult OculusSonarNode::set_config_callback(
  const std::vector<rclcpp::Parameter> & parameters)
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
                case oculus_sonar::OculusSonar_8bits:
                    break;
                case oculus_sonar::OculusSonar_16bits:
                    currentConfig.flags |= 0x02;
                    break;
                default:break;
            }
        } else if (param.get_name() == "nbeams") {
            switch(param.as_int())
            {
                case oculus_sonar::OculusSonar_256beams:
                    break;
                case oculus_sonar::OculusSonar_512beams:
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

    if(currentConfig.frequency_mode   != feedback.masterMode)
    {
        result.successful = false;
        result.reason.append("frequency_mode ");
    }
    //currentConfig.ping_rate      != feedback.pingRate // is broken (?) sonar side
    if(currentConfig.data_depth       != (feedback.flags & 0x02) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("data_depth ");
    }
    if(currentConfig.send_gain        != (feedback.flags & 0x04) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("send_gain ");
    }
    if(currentConfig.gain_assist        != (feedback.flags & 0x10) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("gain_assist ");
    }
    if(currentConfig.nbeams           != (feedback.flags & 0x40) ? 1 : 0)
    {
        result.successful = false;
        result.reason.append("nbeams ");
    }
    if(currentConfig.range            != feedback.range)
    {
        result.successful = false;
        result.reason.append("range ");
    }
    if(currentConfig.gamma_correction != feedback.gammaCorrection)
    {
        result.successful = false;
        result.reason.append("gamma_correction ");
    }
    if(currentConfig.gain_percent     != feedback.gainPercent)
    {
        result.successful = false;
        result.reason.append("gain_percent ");
    }
    if(currentConfig.sound_speed      != feedback.speedOfSound)
    {
        result.successful = false;
        result.reason.append("sound_speed ");
    }
    if(currentConfig.salinity         != feedback.salinity)
    {
        result.successful = false;
        result.reason.append("salinity ");
    }
    //currentConfig.full_ping        != (feedback.flags & 0x08) ? 1 : 0
    
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