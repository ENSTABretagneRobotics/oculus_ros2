#ifndef _NARVAL_OCULUS_CLIENT_H_
#define _NARVAL_OCULUS_CLIENT_H_

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

class Client
{
    public:

    using Socket   = boost::asio::ip::tcp::socket;
    using EndPoint = boost::asio::ip::tcp::endpoint;

    protected:

    Socket   socket_;
    EndPoint remote_;
    uint16_t sourceDevice_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;
    
    OculusSimpleFireMessage requestedFireConfig_;

    OculusMessageHeader    initialHeader_;

    OculusSimplePingResult pingResult_;
    std::vector<uint8_t>   pingData_;

    std::vector<uint8_t> flushedData_;

    void check_reception(const boost::system::error_code& err);
    
    public:

    Client(boost::asio::io_service& service);

    bool is_valid(const OculusMessageHeader& header);
    bool connected() const;
    void send_fire_config(const OculusSimpleFireMessage& fireMsg);

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
};

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_CLIENT_H_
