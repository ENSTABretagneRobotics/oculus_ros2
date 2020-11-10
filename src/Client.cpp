#include <narval_oculus/Client.h>

namespace narval { namespace oculus {

Client::Client(boost::asio::io_service& service) :
    socket_(service),
    remote_(),
    statusListener_(service),
    statusCallbackId_(statusListener_.add_callback(&Client::on_first_status, this))
{
}

void Client::on_first_status(const OculusStatusMsg& msg)
{
    statusListener_.remove_callback(statusCallbackId_);
    
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
}

bool Client::connected() const
{
    return socket_.is_open();
}

}; //namespace oculus
}; //namespace narval

