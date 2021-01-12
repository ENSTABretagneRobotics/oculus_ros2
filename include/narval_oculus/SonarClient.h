#ifndef _NARVAL_OCULUS_SONAR_CLIENT_H_
#define _NARVAL_OCULUS_SONAR_CLIENT_H_

#include <iostream>
#include <cstring>
#include <cmath>

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
    using DummyCallbacks   = CallbackQueue<const OculusMessageHeader&>;
    using MessageCallbacks = CallbackQueue<const OculusMessageHeader&,
                                           const std::vector<uint8_t>&>;

    protected:

    Socket   socket_;
    EndPoint remote_;
    uint16_t sonarId_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;

    OculusMessageHeader    initialHeader_;
    std::vector<uint8_t>   data_;

    PingCallbacks    pingCallbacks_;
    DummyCallbacks   dummyCallbacks_;
    MessageCallbacks messageCallbacks_; // will be called on every received message.

    PingConfig currentConfig_;

    bool         isStandingBy_;
    PingRateType lastPingRate_;

    // helper stubs
    void check_reception(const boost::system::error_code& err);

    public:

    SonarClient(boost::asio::io_service& service);

    bool is_valid(const OculusMessageHeader& header);
    bool connected() const;

    bool send_fire_config(PingConfig fireMsg);
    PingConfig request_fire_config(const PingConfig& fireMsg);
    PingConfig current_fire_config();

    // Stanby mode (saves current ping rate and set it to 0 on the sonar
    void standby();
    void resume();

    // initialization states
    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);

    // main loop begin
    void initiate_receive();
    void receive_callback(const boost::system::error_code err,
                          std::size_t receivedByteCount);

    /////////////////////////////////////////////
    // All remaining member function are related to callbacks and are merely
    // helpers to add callbacks.

    // status callbacks managing functions
    template <typename F, class... Args>
    unsigned int add_status_callback(F&& func, Args&&... args);
    unsigned int add_status_callback(const StatusListener::CallbackT& callback);
    bool remove_status_callback(unsigned int callbackId);
    template <typename F, class... Args>
    bool on_next_status(F&& func, Args&&... args);
    bool on_next_status(const StatusListener::CallbackT& callback);

    // ping callbacks managing functions
    template <typename F, class... Args>
    unsigned int add_ping_callback(F&& func, Args&&... args);
    unsigned int add_ping_callback(const PingCallbacks::CallbackT& callback);
    bool remove_ping_callback(unsigned int callbackId);
    // these are synchronous function which will wait for the next callback call.
    template <typename F, class... Args>
    bool on_next_ping(F&& func, Args&&... args);
    bool on_next_ping(const PingCallbacks::CallbackT& callback);
    
    // dummy message callbacks managing functions
    template <typename F, class... Args>
    unsigned int add_dummy_callback(F&& func, Args&&... args);
    unsigned int add_dummy_callback(const DummyCallbacks::CallbackT& callback);
    bool remove_dummy_callback(unsigned int callbackId);
    // these are synchronous function which will wait for the next callback call.
    template <typename F, class... Args>
    bool on_next_dummy(F&& func, Args&&... args);
    bool on_next_dummy(const DummyCallbacks::CallbackT& callback);
    
    // generic message callbacks managing functions (will be called on any
    // message received).
    template <typename F, class... Args>
    unsigned int add_message_callback(F&& func, Args&&... args);
    unsigned int add_message_callback(const MessageCallbacks::CallbackT& callback);
    bool remove_message_callback(unsigned int callbackId);
    // these are synchronous function which will wait for the next callback call.
    template <typename F, class... Args>
    bool on_next_message(F&& func, Args&&... args);
    bool on_next_message(const MessageCallbacks::CallbackT& callback);
};

// status callbacks
template <typename F, class... Args>
unsigned int SonarClient::add_status_callback(F&& func, Args&&... args)
{
    return statusListener_.add_callback(func, args...);
}

template <typename F, class... Args>
bool SonarClient::on_next_status(F&& func, Args&&... args)
{
    return statusListener_.on_next_status(func, args...);
}

// ping callbacks
template <typename F, class... Args>
unsigned int SonarClient::add_ping_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_ping_callback(static_cast<const PingCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

template <typename F, class... Args>
bool SonarClient::on_next_ping(F&& func, Args&&... args)
{
    return this->on_next_ping(static_cast<const PingCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

// dummy callbacks
template <typename F, class... Args>
unsigned int SonarClient::add_dummy_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_dummy_callback(static_cast<const DummyCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1)));
}

template <typename F, class... Args>
bool SonarClient::on_next_dummy(F&& func, Args&&... args)
{
    return this->on_next_dummy(static_cast<const DummyCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1)));
}

// generic messages callbacks
template <typename F, class... Args>
unsigned int SonarClient::add_message_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_message_callback(static_cast<const MessageCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

template <typename F, class... Args>
bool SonarClient::on_next_message(F&& func, Args&&... args)
{
    return this->on_next_message(static_cast<const MessageCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_SONAR_CLIENT_H_
