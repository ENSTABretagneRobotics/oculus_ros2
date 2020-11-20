#include <narval_oculus/StatusListener.h>

namespace narval { namespace oculus {

StatusListener::StatusListener(boost::asio::io_service& service,
                               unsigned short listeningPort) :
    socket_(service),
    remote_(boost::asio::ip::address_v4::any(), listeningPort)
{
    boost::system::error_code err;
    socket_.open(boost::asio::ip::udp::v4(), err);

    //socket_.set_option(boost::asio::socket_base::broadcast(true));
    if(err)
        throw std::runtime_error("oculus::StatusListener : Error opening socket");

    socket_.bind(remote_);
    if(err)
        throw std::runtime_error("oculus::StatusListener : Socket remote error");

    std::cout << "oculus::StatusListener : listening to remote : " << remote_ << std::endl;
    this->get_one_message();
}

void StatusListener::get_one_message()
{
    socket_.async_receive(boost::asio::buffer(static_cast<void*>(&msg_), sizeof(msg_)),
                          boost::bind(&StatusListener::message_callback, this, _1, _2));
}

StatusListener::CallbackId StatusListener::add_callback(const CallbackT& callback)
{
    return callbacks_.add_callback(callback);
}

bool StatusListener::remove_callback(CallbackId index)
{
    return callbacks_.remove_callback(index);
}

void StatusListener::message_callback(const boost::system::error_code& err,
                                      std::size_t bytesReceived)
{
    if(err) {
        std::cerr << "oculus::StatusListener::read_callback : Status reception error.\n";
        return;
    }

    if(bytesReceived != sizeof(OculusStatusMsg)) {
        std::cerr << "oculus::StatusListener::read_callback : not enough bytes.\n";
        return;
    }
    
    // we are clean here
    //std::cout << msg_ << std::endl;
    callbacks_.call(msg_);
    this->get_one_message();
}

bool StatusListener::on_next_status(const CallbackT& callback)
{
    return callbacks_.add_single_shot(callback);
}

}; //namespace oculus
}; //namespace narval
