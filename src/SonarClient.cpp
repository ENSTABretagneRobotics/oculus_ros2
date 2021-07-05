#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

SonarClient::SonarClient(boost::asio::io_service& service,
                         const Duration& checkerPeriod) :
    socket_(service),
    remote_(),
    sonarId_(0),
    checkerPeriod_(checkerPeriod),
    checkerTimer_(service, checkerPeriod_),
    statusListener_(service),
    statusCallbackId_(0),
    data_(0),
    isStandingBy_(false)
{
    std::memset(&initialHeader_, 0, sizeof(initialHeader_));
    std::memset(&currentConfig_, 0, sizeof(currentConfig_));

    this->initiate_connection();

    this->checkerTimer_.async_wait(
        std::bind(&SonarClient::checker_callback, this, std::placeholders::_1));
}

bool SonarClient::is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sonarId_;
}

bool SonarClient::connected() const
{
    // Not reliable
    return socket_.is_open();
}

bool SonarClient::send_fire_config(PingConfig fireConfig)
{
    fireConfig.head.oculusId    = OCULUS_CHECK_ID;
    fireConfig.head.msgId       = messageSimpleFire;
    fireConfig.head.srcDeviceId = 0;
    fireConfig.head.dstDeviceId = sonarId_;
    fireConfig.head.payloadSize = sizeof(PingConfig) - sizeof(OculusMessageHeader);

    // Other non runtime-configurable parameters (TODO : make then launch parameters)
    fireConfig.networkSpeed = 0xff;


    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&fireConfig), sizeof(fireConfig));
    
    auto bytesSent = socket_.send(buf.data());
    if(bytesSent != sizeof(fireConfig)) {
        std::cerr << "Could not send whole fire message(" << bytesSent
                  << "/" << sizeof(fireConfig) << ")" << std::endl;
        return false;
    }
    return true;
}

SonarClient::PingConfig SonarClient::request_fire_config(const PingConfig& requested)
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
    //std::cout << "Count is : " << count << std::endl << std::flush;

    if(count >= maxCount) {
        std::cerr << "Could not get a proper feedback from the sonar."
                  << "Assuming the configuration is ok (fix this)" << std::endl;
        feedback = requested;
        feedback.head.msgId = 0; // invalid, will be checkable.
    }
    
    return feedback;
}

SonarClient::PingConfig SonarClient::current_fire_config()
{
    // /!\ The header of the returned config will be the header of the carrying
    // message, not the one of a PingConfig.
    PingConfig config;
    bool gotMessage = false;
    this->on_next_message([&](const OculusMessageHeader& header,
                              const std::vector<uint8_t>& data) {
        switch(header.msgId) {
            case messageSimplePingResult:
                config = *(reinterpret_cast<const OculusSimpleFireMessage*>(&header));
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

void SonarClient::standby()
{
    if(isStandingBy_)
        return;
    
    auto request = currentConfig_;

    lastPingRate_ = request.pingRate;
    request.pingRate = pingRateStandby;
    
    this->send_fire_config(request);
}

void SonarClient::resume()
{
    if(!isStandingBy_)
        return;

    auto request = currentConfig_;

    request.pingRate = lastPingRate_;

    this->send_fire_config(request);
}

/**
 * Connection Watchdog.
 *
 * This is an asynchronous loop independent from the connection status (it is
 * looping even if there is no connection). It checks the status of the
 * connection with the sonar and restarts it if necessary.
 */
void SonarClient::checker_callback(const boost::system::error_code& err)
{
    if(!this->connected()) {
        std::cout << "Not connected" << std::endl << std::flush;
    }
    this->checkerTimer_.expires_from_now(checkerPeriod_);
    this->checkerTimer_.async_wait(
        std::bind(&SonarClient::checker_callback, this, std::placeholders::_1));
}

void SonarClient::check_reception(const boost::system::error_code& err)
{
    // no real handling for now
    if(err) {
        std::ostringstream oss;
        oss << "oculue::SonarClient, reception error : " << err;
        throw oss.str();
    }
}

void SonarClient::initiate_connection()
{
    statusCallbackId_ = statusListener_.add_callback(&SonarClient::on_first_status, this);
}

void SonarClient::on_first_status(const OculusStatusMsg& msg)
{
    // got a status message. No need to keep listening.
    statusListener_.remove_callback(statusCallbackId_);
    
    // device id and ip fetched from status message
    sonarId_ = msg.hdr.srcDeviceId;
    remote_ = remote_from_status<EndPoint>(msg);

    // attempting connection
    socket_.async_connect(remote_, boost::bind(&SonarClient::on_connect, this, _1));
}

void SonarClient::on_connect(const boost::system::error_code& err)
{
    if(err) {
        std::ostringstream oss;
        oss << "oculus::SonarClient : connection failure. ( " << remote_ << ")";
        throw std::runtime_error(oss.str());
    }
    std::cout << "Connection successful (" << remote_ << ")" << std::endl << std::flush;
    
    // this enters the ping data reception loop
    this->initiate_receive();

    // this makes the oculus fire pings right away.
    this->send_fire_config(default_fire_config());
}

void SonarClient::initiate_receive()
{
    // asynchronously scan input until finding a valid header.
    // /!\ To be checked : This function and its callback handle the data
    // synchronization with the start of the ping message. It is relying on the
    // assumption that if the data left in the socket is less than the size of
    // the header, a short read will happen, so that the next read will be
    // exacly aligned on the next header.
    static unsigned int count = 0;
    //std::cout << "Initiate receive : " << count << std::endl << std::flush;
    count++;
    socket_.async_receive(
        boost::asio::buffer(reinterpret_cast<uint8_t*>(&initialHeader_), 
                            sizeof(initialHeader_)),
        boost::bind(&SonarClient::receive_callback, this, _1, _2));
}

void SonarClient::receive_callback(const boost::system::error_code err,
                                   std::size_t receivedByteCount)
{

    static unsigned int count = 0;
    //std::cout << "Receive callback : " << count << std::endl << std::flush;
    count++;
    // This function received only enough bytes for an OculusMessageHeader.  If
    // the header is valid, the control is dispatched to the next state
    // depending on the header content (message type). For now only simple ping
    // is implemented, but it seems to be the only message sent by the Oculus.
    // (TODO : check this last statement. Checked : wrong. Other messages seems
    // to be sent but are not documented).
    this->check_reception(err);
    if(receivedByteCount != sizeof(initialHeader_) || !this->is_valid(initialHeader_)) {
        // Either we got data in the middle of a ping or did not get enougth
        // bytes (end of message). Continue listening to get a valid header.
        //std::cout << "Header reception error" << std::endl << std::flush;
        this->initiate_receive();
        return;
    }

    // Messsage header is valid. Now getting the remaining part of the message.
    // (The header contains the payload size, we can receive everything and
    // parse afterwards).  This performs a synchonous read on the socket (block
    // until all is received) (TODO check the timeout)
    data_.resize(sizeof(initialHeader_) + initialHeader_.payloadSize);
    auto byteCount = boost::asio::read(socket_,
        boost::asio::buffer(data_.data() + sizeof(initialHeader_), initialHeader_.payloadSize));
    if(byteCount != initialHeader_.payloadSize) {
        // We did not get enough bytes. Reinitiating reception.
        std::cout << "Data reception error" << std::endl << std::flush;
        this->initiate_receive();
        return;
    }

    // We did received everything. copying initial header in data_ to have a
    // full message, then dispatching.
    *(reinterpret_cast<OculusMessageHeader*>(data_.data())) = initialHeader_;
    
    // Calling generic message callbacks first (in case we want to do something
    // before calling the specialized callbacks).
    messageCallbacks_.call(*(reinterpret_cast<const OculusMessageHeader*>(data_.data())), data_);
    switch(initialHeader_.msgId) {
        case messageSimplePingResult:
            currentConfig_ = reinterpret_cast<const PingResult*>(data_.data())->fireMessage;
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
            pingCallbacks_.call(*(reinterpret_cast<const PingResult*>(data_.data())), data_);
            break;
        case messageDummy:
            currentConfig_.pingRate = pingRateStandby;
            isStandingBy_ = true;
            dummyCallbacks_.call(*(reinterpret_cast<const OculusMessageHeader*>(data_.data())));
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

    // looping
    //std::cout << "Looping" << std::endl << std::flush;
    this->initiate_receive();
}

// status callbacks
unsigned int SonarClient::add_status_callback(const StatusListener::CallbackT& callback)
{
    return statusListener_.add_callback(callback);
}

bool SonarClient::remove_status_callback(unsigned int callbackId)
{
    return statusListener_.remove_callback(callbackId);
}

bool SonarClient::on_next_status(const StatusListener::CallbackT& callback)
{
    return statusListener_.on_next_status(callback);
}

// ping callbacks
unsigned int SonarClient::add_ping_callback(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_callback(callback);
}

bool SonarClient::remove_ping_callback(unsigned int callbackId)
{
    return pingCallbacks_.remove_callback(callbackId);
}

bool SonarClient::on_next_ping(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_single_shot(callback);
}

// dummy callbacks
unsigned int SonarClient::add_dummy_callback(const DummyCallbacks::CallbackT& callback)
{
    return dummyCallbacks_.add_callback(callback);
}

bool SonarClient::remove_dummy_callback(unsigned int callbackId)
{
    return dummyCallbacks_.remove_callback(callbackId);
}

bool SonarClient::on_next_dummy(const DummyCallbacks::CallbackT& callback)
{
    return dummyCallbacks_.add_single_shot(callback);
}

// message callbacks
unsigned int SonarClient::add_message_callback(const MessageCallbacks::CallbackT& callback)
{
    return messageCallbacks_.add_callback(callback);
}

bool SonarClient::remove_message_callback(unsigned int callbackId)
{
    return messageCallbacks_.remove_callback(callbackId);
}

bool SonarClient::on_next_message(const MessageCallbacks::CallbackT& callback)
{
    return messageCallbacks_.add_single_shot(callback);
}

}; //namespace oculus
}; //namespace narval

