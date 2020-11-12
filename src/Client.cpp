#include <narval_oculus/Client.h>

namespace narval { namespace oculus {

Client::Client(boost::asio::io_service& service) :
    socket_(service),
    remote_(),
    sourceDevice_(0),
    statusListener_(service),
    statusCallbackId_(statusListener_.add_callback(&Client::on_first_status, this))
{}

void Client::send_config(const OculusSimpleFireMessage& config)
{
    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&config), sizeof(config));

    auto res = socket_.send(buf.data());
    std::cout << "Sent config (" << res << "/" << sizeof(config) << ")" << std::endl;
}

bool Client::is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sourceDevice_;
}

bool Client::connected() const
{
    return socket_.is_open();
}

void Client::on_first_status(const OculusStatusMsg& msg)
{
    statusListener_.remove_callback(statusCallbackId_);

    sourceDevice_ = msg.hdr.srcDeviceId;
    
    remote_ = Client::remote_from_status(msg);
    socket_.async_connect(remote_, boost::bind(&Client::on_connect, this, _1));
}

void Client::on_connect(const boost::system::error_code& err)
{
    if(err) {
        std::ostringstream oss;
        oss << "oculus::Client : connection failure. ( " << remote_ << ")";
        throw std::runtime_error(oss.str());
    }
    std::cout << "Connection successful (" << remote_ << ")" << std::endl;

    //NOT optional, default config is in Ping message:
    this->send_config(default_configuration());

    this->initiate_receive();
}

void Client::initiate_receive()
{
    // asynchronously scan input until finding a valid header.
    //socket_.async_receive(
    //    boost::asio::buffer(reinterpret_cast<uint8_t*>(&initialHeader_), 4),
    //    boost::bind(&Client::initiate_callback, this, _1, _2));
    socket_.async_receive(
        boost::asio::buffer(reinterpret_cast<uint8_t*>(&initialHeader_), 
                            sizeof(initialHeader_)),
        boost::bind(&Client::initiate_callback, this, _1, _2));
}

void Client::initiate_callback(const boost::system::error_code err,
                               std::size_t receivedByteCount)
{
    // This function receives only the first 4 bytes of a header and checks it.
    // if the 4 bytes are valid, the remaining part of the header is read from
    // the socket.  Then, the control is dispatched to the next state depending
    // on the header content (message type). For now only simple ping is
    // implemented, but it seems to be the only message sent by the Oculus.
    if(err) {
        std::ostringstream oss;
        oss << "Reception error : " << err;
        throw std::runtime_error(oss.str());
    }
    //if(receivedByteCount != 4) {
    //    std::cerr << "\n\n\n\n\n\n\nGot byte issue" << std::endl;
    //    this->initiate_receive();
    //    return;
    //}
    if(receivedByteCount != sizeof(initialHeader_)) {
        //std::cerr << "\n\n\n\n\n\n\nGot byte issue" << std::endl;
        std::cout << std::endl;
        std::cout << "invalid" << std::endl;
        this->initiate_receive();
        return;
    }
    //if(receivedByteCount != sizeof(initialHeader_)) {
    //    std::cout << "got size error " << receivedByteCount << std::endl;
    //}
    
    if(!this->is_valid(initialHeader_)) {
        // if not a valid header, keep listening to get a valid one
        //std::cout << receivedByteCount << " invalid" << "\r" << std::flush;
        //std::cout << receivedByteCount << " invalid" << std::endl;
        std::cout << std::endl;
        std::cout << "invalid" << std::endl;
        this->initiate_receive();
        return;
    }
    //std::cout << receivedByteCount << "======== valid" << std::endl;

    //// reading the remaining parts of the header
    //if(sizeof(initialHeader_) - 4 != boost::asio::read(socket_, boost::asio::buffer(
    //        reinterpret_cast<uint8_t*>(&initialHeader_) + 4,
    //        sizeof(initialHeader_) - 4)))
    //{
    //    std::cerr << "oculus::Client : error receiving header" << std::endl;
    //    this->initiate_receive();
    //    return;
    //}

    // messsage header is valid. Now parsing rest of the message to receive data.
    switch(initialHeader_.msgId) {
        case messageSimpleFire:
            std::cerr << "messageSimpleFire parsing not implemented." << std::endl;
            this->initiate_receive();
            break;
        case messagePingResult:
            std::cerr << "messagePingResult parsing not implemented." << std::endl;
            this->initiate_receive();
            break;
        case messageSimplePingResult:
            //std::cout << "got simple ping" << std::endl;
            this->simple_ping_metadata_receive();
            //this->initiate_receive();
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

void Client::simple_ping_metadata_receive()
{
    // we got the message header in the last sta
    pingResult_.fireMessage.head = initialHeader_;
    //socket_.async_receive(
    boost::asio::async_read(socket_,
        boost::asio::buffer(reinterpret_cast<char*>((&pingResult_)) + sizeof(OculusMessageHeader), 
                            sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader)),
        boost::bind(&Client::simple_ping_metadata_callback, this, _1, _2));
}

void Client::simple_ping_metadata_callback(const boost::system::error_code err,
                                           std::size_t receivedByteCount)
{
    //std::cout << "\rPing count : " << pingResult_.pingId << std::flush;
    
    if(!this->flush_now(pingResult_.imageOffset - sizeof(pingResult_))) {
        this->initiate_receive();
        return;
    }

    this->simple_ping_data_receive();

    //this->flush(pingResult_.messageSize - sizeof(pingResult_));
}

void Client::simple_ping_data_receive()
{
    pingData_.resize(pingResult_.imageSize);
    boost::asio::async_read(socket_,
        boost::asio::buffer(reinterpret_cast<uint8_t*>(pingData_.data()), 
                            pingData_.size()),
        boost::bind(&Client::simple_ping_data_callback, this, _1, _2));
}

void Client::simple_ping_data_callback(const boost::system::error_code err,
                                       std::size_t receivedByteCount)
{
    if(err || receivedByteCount != pingData_.size()) {
        std::cerr << "Error pind data receive" << std::endl;
        this->initiate_receive();
        return;
    }
    std::cout << "\rPing count : " << pingResult_.pingId << std::flush;
    //std::cout << pingResult_ << std::endl;
    this->initiate_receive();
}

void Client::flush(std::size_t byteCount)
{
    flushedData_.resize(byteCount);
    boost::asio::async_read(socket_,
        boost::asio::buffer(reinterpret_cast<uint8_t*>(flushedData_.data()), 
                            flushedData_.size()),
        boost::bind(&Client::flush_callback, this, _1, _2));
}

void Client::flush_callback(const boost::system::error_code err,
                            std::size_t receivedByteCount)
{
    if(err) {
        std::cerr << "Error on flush : " << err << std::endl;
    }

    if(receivedByteCount != flushedData_.size()) {
        std::cerr << "Some data were not flushed ("
                  << receivedByteCount << "/" << flushedData_.size()
                  << ")" << std::endl;
    }
    this->initiate_receive();
}

bool Client::flush_now(std::size_t byteCount)
{
    flushedData_.resize(byteCount);
    return byteCount == boost::asio::read(socket_,
        boost::asio::buffer(flushedData_.data(), flushedData_.size()));
}

}; //namespace oculus
}; //namespace narval

