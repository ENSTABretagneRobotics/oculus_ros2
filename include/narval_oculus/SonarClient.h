#ifndef _NARVAL_OCULUS_SONAR_CLIENT_H_
#define _NARVAL_OCULUS_SONAR_CLIENT_H_

#include <iostream>
#include <cstring>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/print_utils.h>
#include <narval_oculus/utils.h>
#include <narval_oculus/CallbackQueue.h>
#include <narval_oculus/StatusListener.h>

namespace narval { namespace oculus {

class SonarClient
{
    public:

    using Socket        = boost::asio::ip::tcp::socket;
    using EndPoint      = boost::asio::ip::tcp::endpoint;
    using PingConfig    = OculusSimpleFireMessage;
    using PingResult    = OculusSimplePingResult;
    using PingCallbacks = CallbackQueue<const PingResult&,
                                        const std::vector<uint8_t>&>; 

    protected:

    Socket   socket_;
    EndPoint remote_;
    uint16_t sonarId_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;
    
    PingConfig requestedFireConfig_;
    PingConfig currentFireConfig_;

    OculusMessageHeader    initialHeader_;
    
    std::vector<uint8_t>   data_;
    // callbacks to be called when a full ping is received.
    PingCallbacks          pingCallbacks_;

    void check_reception(const boost::system::error_code& err);
    
    public:

    SonarClient(boost::asio::io_service& service);

    PingConfig current_fire_config() const;

    bool is_valid(const OculusMessageHeader& header);
    bool connected() const;
    void send_fire_config(PingConfig& fireMsg);

    // initialization states
    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);

    // main loop begin
    void initiate_receive();
    void receive_callback(const boost::system::error_code err,
                          std::size_t receivedByteCount);

    template <typename F, class... Args>
    unsigned int add_ping_callback(F&& func, Args&&... args);
    unsigned int add_ping_callback(const PingCallbacks::CallbackT& callback);
    bool remove_ping_callback(unsigned int callbackId);
    
    template <typename F, class... Args>
    unsigned int add_status_callback(F&& func, Args&&... args);
    unsigned int add_status_callback(const StatusListener::CallbackT& callback);
    bool remove_status_callback(unsigned int callbackId);
};

template <typename F, class... Args>
unsigned int SonarClient::add_ping_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_ping_callback(static_cast<const PingCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

template <typename F, class... Args>
unsigned int SonarClient::add_status_callback(F&& func, Args&&... args)
{
    return statusListener_.add_callback(func, args...);
}

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_SONAR_CLIENT_H_
