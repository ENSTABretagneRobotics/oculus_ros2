#ifndef _NARVAL_OCULUS_CLIENT_H_
#define _NARVAL_OCULUS_CLIENT_H_

#include <iostream>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/utils.h>
#include <narval_oculus/CallbackQueue.h>
#include <narval_oculus/StatusListener.h>

namespace narval { namespace oculus {

class Client
{
    public:

    using Socket     = boost::asio::ip::tcp::socket;
    using EndPoint   = boost::asio::ip::tcp::endpoint;

    static EndPoint remote_from_status(const OculusStatusMsg& status)
    {
        // going through string conversion allows to not care about
        // endianess. (fix this)
        return EndPoint(boost::asio::ip::address_v4::from_string(
            ip_to_string(status.ipAddr)), 52100);
    }

    protected:

    Socket   socket_;
    EndPoint remote_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;

    public:

    Client(boost::asio::io_service& service);

    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);

    bool connected() const;
};

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_CLIENT_H_
