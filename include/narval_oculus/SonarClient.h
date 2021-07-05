#ifndef _NARVAL_OCULUS_SONAR_CLIENT_H_
#define _NARVAL_OCULUS_SONAR_CLIENT_H_

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <memory>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/utils.h>
#include <narval_oculus/print_utils.h>
#include <narval_oculus/StatusListener.h>

namespace narval { namespace oculus {

/**
 * This is the base class to handle an Oculus sonar.
 *
 * It solely handle the network connection to the sonar. Use a subclass such as
 * SonarDriver to control the sonar or receive data.
 */
class SonarClient
{
    public:

    using IoService    = boost::asio::io_service;
    using IoServicePtr = std::shared_ptr<IoService>;
    using Socket       = boost::asio::ip::tcp::socket;
    using SocketPtr    = std::unique_ptr<Socket>;
    using EndPoint     = boost::asio::ip::tcp::endpoint;
    using Duration     = boost::posix_time::time_duration;

    enum ConnectionState { Initializing, Attempt, Connected, Lost };

    protected:
    
    IoServicePtr    ioService_;
    SocketPtr       socket_;
    EndPoint        remote_;
    uint16_t        sonarId_;
    ConnectionState connectionState_;

    Duration                     checkerPeriod_;
    boost::asio::deadline_timer  checkerTimer_;
    Clock                        clock_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;

    OculusMessageHeader    initialHeader_;
    std::vector<uint8_t>   data_;

    // helper stubs
    void checker_callback(const boost::system::error_code& err);
    void check_reception(const boost::system::error_code& err);

    public:

    SonarClient(const IoServicePtr& ioService,
                const Duration& checkerPeriod = boost::posix_time::seconds(1));

    bool is_valid(const OculusMessageHeader& header);
    bool connected() const;

    // initialization states
    void reset_connection();
    void close_connection();
    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);
    virtual void on_connect();

    // main loop begin
    void initiate_receive();
    void header_received_callback(const boost::system::error_code err,
                                  std::size_t receivedByteCount);
    void data_received_callback(const boost::system::error_code err,
                                std::size_t receivedByteCount);
    
    // This is called regardless of the content of the message.
    // To be reimplemented in a subclass (does nothing by default).
    virtual void handle_message(const OculusMessageHeader& header,
                                const std::vector<uint8_t>& data);

    template <typename T = float>
    T time_since_last_message() const { return clock_.now(); }
};

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_SONAR_CLIENT_H_
