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
    using PingCallbacks = CallbackQueue<const OculusSimplePingResult&,
                                        const std::vector<uint8_t>&>; 

    protected:

    Socket   socket_;
    EndPoint remote_;
    uint16_t sourceDevice_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;
    
    OculusSimpleFireMessage requestedFireConfig_;
    OculusSimpleFireMessage currentFireConfig_;

    OculusMessageHeader    initialHeader_;

    // data structure used for ping reception
    OculusSimplePingResult pingResult_;
    std::vector<uint8_t>   pingData_;
    // callbacks to be called when a full ping is received.
    PingCallbacks          pingCallbacks_;

    // used of discarding bytes
    std::vector<uint8_t> flushedData_;

    void check_reception(const boost::system::error_code& err);
    
    public:

    SonarClient(boost::asio::io_service& service);

    OculusSimpleFireMessage current_fire_config() const;

    bool is_valid(const OculusMessageHeader& header);
    bool connected() const;
    void send_fire_config(OculusSimpleFireMessage& fireMsg);

    // The client is actually a state machine
    // These function represent the states

    // initialization states
    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);

    // main loop begin
    void initiate_receive();
    void initiate_callback(const boost::system::error_code err,
                           std::size_t receivedByteCount);
    
    // OculusSimplePingResult related states.
    void simple_ping_receive_start();
    void simple_ping_metadata_callback(const boost::system::error_code err,
                                       std::size_t receivedByteCount);
    void simple_ping_data_callback(const boost::system::error_code err,
                                   std::size_t receivedByteCount);
    
    // Not states. Utility function to discard bytes in the data stream.
    void flush(std::size_t byteCount);
    void flush_callback(const boost::system::error_code err,
                        std::size_t receivedByteCount);
    bool flush_now(std::size_t byteCount);

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
