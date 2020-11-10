#ifndef _DEF_NARVAL_OCULUS_STATUS_LISTENER_H_
#define _DEF_NARVAL_OCULUS_STATUS_LISTENER_H_

#include <iostream>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/CallbackQueue.h>

namespace narval { namespace oculus {

class StatusListener
{
    public:

    using Socket     = boost::asio::ip::udp::socket;
    using EndPoint   = boost::asio::ip::udp::endpoint;
    using Callbacks  = CallbackQueue<const OculusStatusMsg&>;
    using CallbackT  = Callbacks::CallbackT;
    using CallbackId = Callbacks::CallbackId;

    protected:

    Socket          socket_;
    EndPoint        remote_;
    OculusStatusMsg msg_;
    Callbacks       callbacks_;

    public:

    StatusListener(boost::asio::io_service& service, unsigned short listeningPort = 52102);
    
    template <typename F, class... Args>
    CallbackId add_callback(F&& func, Args&&... args);
    CallbackId add_callback(const CallbackT& callback);
    bool remove_callback(CallbackId index);
    
    private:
    
    void get_one_message();
    void message_callback(const boost::system::error_code& err, std::size_t bytesReceived);
};

template <typename F, class... Args>
StatusListener::CallbackId StatusListener::add_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_callback(static_cast<const CallbackT&>(
        std::bind(func, args..., std::placeholders::_1)));
}

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_STATUS_LISTENER_H_
