#include <narval_oculus/SonarDriver.h>

namespace narval { namespace oculus {

SonarDriver::SonarDriver(const IoServicePtr& service,
                         const Duration& checkerPeriod) :
    SonarClient(service, checkerPeriod),
    isStandingBy_(false),
    currentConfig_(default_fire_config())
{
    //std::memset(&currentConfig_, 0, sizeof(currentConfig_));
}

bool SonarDriver::send_fire_config(PingConfig fireConfig)
{
    // do better -> mutex might be necessary
    if(!this->connected() || !socket_)
        return false;

    fireConfig.head.oculusId    = OCULUS_CHECK_ID;
    fireConfig.head.msgId       = messageSimpleFire;
    fireConfig.head.srcDeviceId = 0;
    fireConfig.head.dstDeviceId = sonarId_;
    fireConfig.head.payloadSize = sizeof(PingConfig) - sizeof(OculusMessageHeader);

    // Other non runtime-configurable parameters (TODO : make then launch parameters)
    fireConfig.networkSpeed = 0xff;

    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&fireConfig), sizeof(fireConfig));
    
    // auto bytesSent = socket_->send(buf.data());
    auto bytesSent = this->send(buf);
    if(bytesSent != sizeof(fireConfig)) {
        std::cerr << "Could not send whole fire message(" << bytesSent
                  << "/" << sizeof(fireConfig) << ")" << std::endl;
        return false;
    }
    return true;
}

SonarDriver::PingConfig SonarDriver::request_fire_config(const PingConfig& requested)
{
    //if(!this->send_fire_config(requested)) return requested;
    //std::cout << "Sending config :\n" << requested << std::endl << std::flush;
   
    // Waiting for a ping or a dummy message to have a feedback on the config changes.
    PingConfig feedback;
    int count = 0;
    const int maxCount = 100; // TODO make a parameter out of this
    do {
        if(this->send_fire_config(requested)) {
            feedback = this->current_fire_config();
            if(check_config_feedback(requested, feedback))
                break;
        }
        count++;
    } while(count < maxCount);
    //std::cout << "Config feedback :\n" << feedback << std::endl << std::flush;
    std::cout << "Count is : " << count << std::endl << std::flush;

    if(count >= maxCount) {
        std::cerr << "Could not get a proper feedback from the sonar."
                  << "Assuming the configuration is ok (fix this)" << std::endl;
        feedback = requested;
        feedback.head.msgId = 0; // invalid, will be checkable.
    }
    
    return feedback;
}

SonarDriver::PingConfig SonarDriver::current_fire_config()
{
    // /!\ The header of the returned config will be the header of the carrying
    // message, not the one of a PingConfig.
    PingConfig config;
    bool gotMessage = false;
    this->on_next_message([&](const OculusMessageHeader& header,
                              const std::vector<uint8_t>& data) {
        switch(header.msgId) {
            case messageSimplePingResult:
                //config = *(reinterpret_cast<const OculusSimpleFireMessage*>(&header));
                config = *(reinterpret_cast<const OculusSimpleFireMessage*>(data.data()));
                gotMessage = true;
                break;
            // messageDummy and all other messages are treated in the same way
            // because only a simple ping result is a valid message to get a
            // configuration.
            case messageDummy:
                config = default_fire_config();
                config.head = header;
                config.pingRate = pingRateStandby;
                gotMessage = true;
                break;
            default:
                config = default_fire_config();
                config.head = header;
                config.pingRate = pingRateStandby;
                break;
        }
    });
    // When masterMode = 2, the sonar force gainPercent between 40& and 100%,
    // BUT still needs resquested gainPercent to be between 0% and 100%. (If
    // you request a gainPercent=0 in masterMode=2, the fireMessage in the ping
    // results will be 40%)The gainPercent is rescaled here to ensure
    // consistent parameter handling on client side).
    if(config.masterMode == 2) {
        config.gainPercent = (config.gainPercent - 40.0) * 100.0 / 60.0;
    }
    return config;
}

void SonarDriver::standby()
{
    if(isStandingBy_)
        return;
    
    auto request = currentConfig_;

    lastPingRate_ = request.pingRate;
    request.pingRate = pingRateStandby;
    
    this->send_fire_config(request);
}

void SonarDriver::resume()
{
    if(!isStandingBy_)
        return;

    auto request = currentConfig_;

    request.pingRate = lastPingRate_;

    this->send_fire_config(request);
}

/**
 * Called when the driver first connection with the sonar.
 *
 * Will be called again if a new connection append.
 */
void SonarDriver::on_connect()
{
    // This makes the oculus fire right away.
    // On first connection currentConfig is equal to default_fire_config().
    this->send_fire_config(currentConfig_);
}

/**
 * Called when a new complete message is received.
 */
void SonarDriver::handle_message(const OculusMessageHeader& header,
                                 const std::vector<uint8_t>& data)
{
    // Calling generic message callbacks first (in case we want to do something
    // before calling the specialized callbacks).
    messageCallbacks_.call(header, data);
    switch(header.msgId) {
        case messageSimplePingResult:
            currentConfig_ = reinterpret_cast<const PingResult*>(data.data())->fireMessage;
            // When masterMode = 2, the sonar force gainPercent between 40& and
            // 100%, BUT still needs resquested gainPercent to be between 0%
            // and 100%. (If you request a gainPercent=0 in masterMode=2, the
            // fireMessage in the ping results will be 40%)The gainPercent is
            // rescaled here to ensure consistent parameter handling on client
            // side).
            if(currentConfig_.masterMode == 2) {
                currentConfig_.gainPercent = (currentConfig_.gainPercent - 40.0) * 100.0 / 60.0;
            }
            isStandingBy_ = false;
            pingCallbacks_.call(*(reinterpret_cast<const PingResult*>(data.data())), data);
            break;
        case messageDummy:
            currentConfig_.pingRate = pingRateStandby;
            isStandingBy_ = true;
            dummyCallbacks_.call(header);
            break;
        case messageSimpleFire:
            std::cerr << "messageSimpleFire parsing not implemented." << std::endl;
            break;
        case messagePingResult:
            std::cerr << "messagePingResult parsing not implemented." << std::endl;
            break;
        case messageUserConfig:
            std::cerr << "messageUserConfig parsing not implemented." << std::endl;
            break;
        default:
            break;
    }
}

// status callbacks
unsigned int SonarDriver::add_status_callback(const StatusListener::CallbackT& callback)
{
    return statusListener_.add_callback(callback);
}

bool SonarDriver::remove_status_callback(unsigned int callbackId)
{
    return statusListener_.remove_callback(callbackId);
}

bool SonarDriver::on_next_status(const StatusListener::CallbackT& callback)
{
    return statusListener_.on_next_status(callback);
}

// ping callbacks
unsigned int SonarDriver::add_ping_callback(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_callback(callback);
}

bool SonarDriver::remove_ping_callback(unsigned int callbackId)
{
    return pingCallbacks_.remove_callback(callbackId);
}

bool SonarDriver::on_next_ping(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_single_shot(callback);
}

// dummy callbacks
unsigned int SonarDriver::add_dummy_callback(const DummyCallbacks::CallbackT& callback)
{
    return dummyCallbacks_.add_callback(callback);
}

bool SonarDriver::remove_dummy_callback(unsigned int callbackId)
{
    return dummyCallbacks_.remove_callback(callbackId);
}

bool SonarDriver::on_next_dummy(const DummyCallbacks::CallbackT& callback)
{
    return dummyCallbacks_.add_single_shot(callback);
}

// message callbacks
unsigned int SonarDriver::add_message_callback(const MessageCallbacks::CallbackT& callback)
{
    return messageCallbacks_.add_callback(callback);
}

bool SonarDriver::remove_message_callback(unsigned int callbackId)
{
    return messageCallbacks_.remove_callback(callbackId);
}

bool SonarDriver::on_next_message(const MessageCallbacks::CallbackT& callback)
{
    return messageCallbacks_.add_single_shot(callback);
}

}; //namespace oculus
}; //namespace narval

