#ifndef _DEF_NARVAL_OCULUS_STATUS_LISTENER_H_
#define _DEF_NARVAL_OCULUS_STATUS_LISTENER_H_

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/print_utils.h>

namespace narval { namespace oculus {

class StatusListener
{
    public:

    using Socket   = boost::asio::ip::udp::socket;
    using EndPoint = boost::asio::ip::udp::endpoint;

    protected:

    Socket          socket_;
    EndPoint        remote_;
    OculusStatusMsg msg_;

    public:

    StatusListener(boost::asio::io_service& service, unsigned short listeningPort = 52102);
    
    private:

    void get_one_message();
    void message_callback(const boost::system::error_code& err, std::size_t bytesReceived);
};


}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_STATUS_LISTENER_H_
