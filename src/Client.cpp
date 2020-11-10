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
        std::runtime_error(oss.str());
    }

    std::cout << "Connection successful (" << remote_ << ")" << std::endl;
    this->send_config(default_configuration());
}

void Client::send_config(const OculusSimpleFireMessage& config)
{
    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&config), sizeof(config));

    auto res = socket_.send(buf.data());
    std::cout << "Sent config (" << res << "/" << sizeof(config) << ")" << std::endl;

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
    if(err) {
        std::ostringstream oss;
        oss << "Reception error : " << err;
        throw std::runtime_error(oss.str());
    }
    
    if(!this->validate_header(initialHeader_)) {
        this->initiate_receive();
        return;
    }

    switch(initialHeader_.msgId) {
        case messageSimpleFire:
            std::cout << "Got simple fire" << std::endl;
            std::cout << initialHeader_ << std::endl;
            break;
        case messagePingResult:
            std::cout << "Got ping" << std::endl;
            std::cout << initialHeader_ << std::endl;
            break;
        case messageSimplePingResult:
            std::cout << "Got simple ping" << std::endl;
            std::cout << initialHeader_ << std::endl;
            break;
        case messageUserConfig:
            std::cout << "Got user config" << std::endl;
            std::cout << initialHeader_ << std::endl;
            break;
        case messageDummy:
            std::cout << "Got dummy" << std::endl;
            std::cout << initialHeader_ << std::endl;
            break;
        default:
            break;
    }

    this->initiate_receive();
}

bool Client::validate_header(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sourceDevice_;
}

bool Client::connected() const
{
    return socket_.is_open();
}

}; //namespace oculus
}; //namespace narval

