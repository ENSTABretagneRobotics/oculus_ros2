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
    std::cout << "Got status" << std::endl;
    statusListener_.remove_callback(statusCallbackId_);
    
    remote_ = Client::remote_from_status(msg);
    std::cout << "Remote : " << remote_ << std::endl << std::flush;
}

}; //namespace oculus
}; //namespace narval

