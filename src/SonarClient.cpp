#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

SonarClient::SonarClient(boost::asio::io_service& service) :
    socket_(service),
    remote_(),
    sonarId_(0),
    statusListener_(service),
    statusCallbackId_(statusListener_.add_callback(&SonarClient::on_first_status, this))
{}

bool SonarClient::is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sonarId_;
}

bool SonarClient::connected() const
{
    return socket_.is_open();
}

bool SonarClient::are_similar(const PingConfig& lhs, const PingConfig& rhs)
{
    // this function compare fields in fire messages which are supposed to be
    // identical between a fire confgi send to the sonar and the fire config
    // contained in the ping result (sadly, not all fields are equivalent).
    return lhs.masterMode       == rhs.masterMode
        && lhs.gammaCorrection  == rhs.gammaCorrection
        && lhs.flags            == rhs.flags
        && lhs.range            == rhs.range
        && lhs.gainPercent      == rhs.gainPercent;
}

SonarClient::PingConfig SonarClient::request_fire_config(PingConfig fireConfig)
{
    // mandatory header filling
    fireConfig.head.oculusId    = OCULUS_CHECK_ID;
    fireConfig.head.msgId       = messageSimpleFire;
    fireConfig.head.srcDeviceId = 0;
    fireConfig.head.dstDeviceId = sonarId_;
    fireConfig.head.payloadSize = sizeof(PingConfig) - sizeof(OculusMessageHeader);

    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&fireConfig), sizeof(fireConfig));
    
    auto bytesSent = socket_.send(buf.data());
    if(bytesSent != sizeof(fireConfig)) {
        std::cerr << "Could not send whole fire message(" << bytesSent
                  << "/" << sizeof(fireConfig) << ")" << std::endl;
        return fireConfig;
    }
    std::cout << "Sent config :\n" << fireConfig << std::endl;

    PingConfig actualSonarConfig;
    std::memset(&actualSonarConfig, 0, sizeof(actualSonarConfig));
    int count = 0;
    do {
        this->on_next_ping([&](const PingResult& metadata, const std::vector<uint8_t>& data) {
            actualSonarConfig = metadata.fireMessage;
        });
        count++;
    } while(count < 20 && !are_similar(actualSonarConfig, fireConfig));
    std::cout << "Actual config :\n" << actualSonarConfig << std::endl;
    std::cout << "Count is : " << count << std::endl;

    return actualSonarConfig;
}

void SonarClient::send_fire_config(PingConfig fireConfig)
{
    fireConfig.head.oculusId    = OCULUS_CHECK_ID;
    fireConfig.head.msgId       = messageSimpleFire;
    fireConfig.head.srcDeviceId = 0;
    fireConfig.head.dstDeviceId = sonarId_;
    fireConfig.head.payloadSize = sizeof(PingConfig) - sizeof(OculusMessageHeader);

    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&fireConfig), sizeof(fireConfig));
    
    auto bytesSent = socket_.send(buf.data());
    if(bytesSent != sizeof(fireConfig)) {
        std::cerr << "Could not send whole fire message(" << bytesSent
                  << "/" << sizeof(fireConfig) << ")" << std::endl;
        return;
    }
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
    std::cout << "Connection successful (" << remote_ << ")" << std::endl;
    
    // this enters the pind data reception loop
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
    socket_.async_receive(
        boost::asio::buffer(reinterpret_cast<uint8_t*>(&initialHeader_), 
                            sizeof(initialHeader_)),
        boost::bind(&SonarClient::receive_callback, this, _1, _2));
}

void SonarClient::receive_callback(const boost::system::error_code err,
                                   std::size_t receivedByteCount)
{
    // This function receives only enough bytes for an OculusMessageHeader.  If
    // the header is valid, the control is dispatched to the next state
    // depending on the header content (message type). For now only simple ping
    // is implemented, but it seems to be the only message sent by the Oculus.
    // (TODO : check this last statement).
    this->check_reception(err);
    if(receivedByteCount != sizeof(initialHeader_) || !this->is_valid(initialHeader_)) {
        // Either we got data in the middle of a ping or did not get enougth
        // bytes (end of message). Continue listening to get a valid header.
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
        this->initiate_receive();
        return;
    }

    // We did received everything. copying initial header in data_ to have a
    // full message, then dispatching.
    *(reinterpret_cast<OculusMessageHeader*>(data_.data())) = initialHeader_;
    switch(initialHeader_.msgId) {
        case messageSimplePingResult:
            pingCallbacks_.call(*(reinterpret_cast<const PingResult*>(data_.data())), data_);
            break;
        case messageDummy:
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
    this->initiate_receive();
}

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

}; //namespace oculus
}; //namespace narval

