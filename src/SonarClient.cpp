#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

SonarClient::SonarClient(boost::asio::io_service& service) :
    socket_(service),
    remote_(),
    sourceDevice_(0),
    statusListener_(service),
    statusCallbackId_(statusListener_.add_callback(&SonarClient::on_first_status, this)),
    requestedFireConfig_(default_fire_config())
{}

bool SonarClient::is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sourceDevice_;
}

bool SonarClient::connected() const
{
    return socket_.is_open();
}

void SonarClient::send_fire_config(const OculusSimpleFireMessage& fireConfig)
{
    // This function changes the ping configuration of the oculus and make it
    // start firing pings.
    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&fireConfig), sizeof(fireConfig));
    
    auto bytesSent = socket_.send(buf.data());
    if(bytesSent != sizeof(fireConfig)) {
        std::cerr << "Could not send whole fire message(" << bytesSent
                  << "/" << sizeof(fireConfig) << ")" << std::endl;
        return;
    }
    requestedFireConfig_ = fireConfig;
    std::cout << "Fire message sent." << std::endl;
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
    sourceDevice_ = msg.hdr.srcDeviceId;
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

    // this makes the oculus fire pings right away.
    this->send_fire_config(requestedFireConfig_);
    
    // this enters the pind data reception loop
    this->initiate_receive();
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
        boost::bind(&SonarClient::initiate_callback, this, _1, _2));
}

void SonarClient::initiate_callback(const boost::system::error_code err,
                               std::size_t receivedByteCount)
{
    // This function receives only the first 4 bytes of a header and checks it.
    // if the 4 bytes are valid, the remaining part of the header is read from
    // the socket.  Then, the control is dispatched to the next state depending
    // on the header content (message type). For now only simple ping is
    // implemented, but it seems to be the only message sent by the Oculus.
    this->check_reception(err);
    if(receivedByteCount != sizeof(initialHeader_) || !this->is_valid(initialHeader_)) {
        // Either we got data in the middle of a ping or did not get enougth
        // bytes (end of message). Continue listening to get a valid header.
        this->initiate_receive();
        return;
    }

    // Messsage header is valid. Now identifying the message type to continue
    // reception.
    switch(initialHeader_.msgId) {
        case messageSimplePingResult: // only valid option
            this->simple_ping_receive_start();
            break;
        case messageSimpleFire:
            std::cerr << "messageSimpleFire parsing not implemented." << std::endl;
            this->initiate_receive();
            break;
        case messagePingResult:
            std::cerr << "messagePingResult parsing not implemented." << std::endl;
            this->initiate_receive();
            break;
        case messageUserConfig:
            std::cerr << "messageUserConfig parsing not implemented." << std::endl;
            this->initiate_receive();
            break;
        case messageDummy:
            std::cerr << "messageDummy parsing not implemented." << std::endl;
            this->initiate_receive();
            break;
        default:
            this->initiate_receive();
            break;
    }
}

void SonarClient::simple_ping_receive_start()
{
    // We got the message header in the initiate_receive state. We now know
    // that we will get an OculusSimplePingResult.
    pingResult_.fireMessage.head = initialHeader_;

    // Now fetching the rest of the metadata to deduce the size to the data to receive.
    boost::asio::async_read(socket_,
        boost::asio::buffer(reinterpret_cast<char*>((&pingResult_)) + sizeof(OculusMessageHeader), 
                            sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader)),
        boost::bind(&SonarClient::simple_ping_metadata_callback, this, _1, _2));
}

void SonarClient::simple_ping_metadata_callback(const boost::system::error_code err,
                                           std::size_t receivedByteCount)
{
    this->check_reception(err);
    if(receivedByteCount != sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader)) {
        std::cerr << "git not receive enough bytes for simple ping metadata" << std::endl;
        this->initiate_receive();
    }
    
    // We received the ping metadata. There are some unused bytes between the
    // metadata and the data in the ping data stream. Discarding them.
    if(!this->flush_now(pingResult_.imageOffset - sizeof(pingResult_))) {
        std::cerr << "Did not flush enough offset bytes for ping" << std::endl;
        this->initiate_receive();
        return;
    }

    // unsued bytes successfully dicarded. Now fetching data
    //this->simple_ping_data_receive();
    pingData_.resize(pingResult_.imageSize);
    boost::asio::async_read(socket_,
        boost::asio::buffer(reinterpret_cast<uint8_t*>(pingData_.data()), 
                            pingData_.size()),
        boost::bind(&SonarClient::simple_ping_data_callback, this, _1, _2));
}

void SonarClient::simple_ping_data_callback(const boost::system::error_code err,
                                       std::size_t receivedByteCount)
{
    this->check_reception(err);
    if(receivedByteCount != pingData_.size()) {
        std::cerr << "Did not receive enough byte for ping data." << std::endl;
        this->initiate_receive();
        return;
    }
    // we got a full ping
    pingCallbacks_.call(pingResult_, pingData_);
    this->initiate_receive();
}

void SonarClient::flush(std::size_t byteCount)
{
    flushedData_.resize(byteCount);
    boost::asio::async_read(socket_,
        boost::asio::buffer(reinterpret_cast<uint8_t*>(flushedData_.data()), 
                            flushedData_.size()),
        boost::bind(&SonarClient::flush_callback, this, _1, _2));
}

void SonarClient::flush_callback(const boost::system::error_code err,
                            std::size_t receivedByteCount)
{
    this->check_reception(err);
    if(receivedByteCount != flushedData_.size()) {
        std::cerr << "Some data were not flushed ("
                  << receivedByteCount << "/" << flushedData_.size()
                  << ")" << std::endl;
    }
    this->initiate_receive();
}

bool SonarClient::flush_now(std::size_t byteCount)
{
    flushedData_.resize(byteCount);
    return byteCount == boost::asio::read(socket_,
        boost::asio::buffer(flushedData_.data(), flushedData_.size()));
}

unsigned int SonarClient::add_ping_callback(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_callback(callback);
}

bool SonarClient::remove_ping_callback(unsigned int callbackId)
{
    return pingCallbacks_.remove_callback(callbackId);
}

}; //namespace oculus
}; //namespace narval

