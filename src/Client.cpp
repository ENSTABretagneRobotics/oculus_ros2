#include <narval_oculus/Client.h>

namespace narval { namespace oculus {

Client::Client(boost::asio::io_service& service) :
    socket_(service),
    remote_(),
    statusListener_(service),
    statusCallbackId_(statusListener_.add_callback(&Client::on_first_status, this)),
    sourceDevice_(0)
{
}

void Client::send_config(const OculusSimpleFireMessage& config)
{
    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&config), sizeof(config));

    auto res = socket_.send(buf.data());
    std::cout << "Sent config (" << res << "/" << sizeof(config) << ")" << std::endl;
}

bool Client::validate_header(const OculusMessageHeader& header)
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

    //optional, default config is in Ping message:
    //this->send_config(default_configuration());

    this->initiate_receive();
}

void Client::initiate_receive()
{
    socket_.async_receive(
        boost::asio::buffer(reinterpret_cast<void*>(&initialHeader_), sizeof(initialHeader_)),
        boost::bind(&Client::initiate_callback, this, _1, _2));
}

void Client::initiate_callback(const boost::system::error_code err,
                               std::size_t receivedByteCount)
{
    // This function receives only messages header and dispatch control to the
    // next state depending on the header content (message type).  For now only
    // simple ping is implemented, but it seems to be the only message sent by
    // the Oculus.
    if(err) {
        std::ostringstream oss;
        oss << "Reception error : " << err;
        throw std::runtime_error(oss.str());
    }
    
    if(!this->validate_header(initialHeader_)) {
        // if not a valid header, keep listening to get a valid one
        this->initiate_receive();
        return;
    }

    // messsage header is valid. Now parsing rest of the message to receive data.
    switch(initialHeader_.msgId) {
        case messageSimpleFire:
            std::cerr << "messageSimpleFire parsing not implemented." << std::endl;
            std::cerr << initialHeader_ << std::endl;
            break;
        case messagePingResult:
            std::cerr << "messagePingResult parsing not implemented." << std::endl;
            std::cerr << initialHeader_ << std::endl;
            break;
        case messageSimplePingResult:
            this->simple_ping_metadata_receive();
            break;
        case messageUserConfig:
            std::cerr << "messageUserConfig parsing not implemented." << std::endl;
            std::cerr << initialHeader_ << std::endl;
            break;
        case messageDummy:
            std::cerr << "messageDummy parsing not implemented." << std::endl;
            std::cerr << initialHeader_ << std::endl;
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
    socket_.async_receive(
        boost::asio::buffer(reinterpret_cast<char*>((&pingResult_)) + sizeof(OculusMessageHeader), 
                            sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader)),
        boost::bind(&Client::simple_ping_metadata_callback, this, _1, _2));
}

void Client::simple_ping_metadata_callback(const boost::system::error_code err,
                                           std::size_t receivedByteCount)
{
    std::cout << "ping metadata (received : " 
              << receivedByteCount << "/"
              << sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader) << ")\n";
    std::cout << pingResult_ << std::endl << std::endl;
    this->initiate_receive();
}

}; //namespace oculus
}; //namespace narval

