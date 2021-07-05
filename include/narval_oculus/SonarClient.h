#ifndef _NARVAL_OCULUS_SONAR_CLIENT_H_
#define _NARVAL_OCULUS_SONAR_CLIENT_H_

#include <iostream>
#include <cstring>
#include <cmath>

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

    using Socket   = boost::asio::ip::tcp::socket;
    using EndPoint = boost::asio::ip::tcp::endpoint;
    using Duration = boost::posix_time::time_duration;

    protected:

    Socket   socket_;
    EndPoint remote_;
    uint16_t sonarId_;

    Duration                     checkerPeriod_;
    boost::asio::deadline_timer  checkerTimer_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;

    OculusMessageHeader    initialHeader_;
    std::vector<uint8_t>   data_;

    // helper stubs
    void checker_callback(const boost::system::error_code& err);
    void check_reception(const boost::system::error_code& err);

    public:

    SonarClient(boost::asio::io_service& service,
                const Duration& checkerPeriod = boost::posix_time::seconds(1));

    bool is_valid(const OculusMessageHeader& header);

    // initialization states
    void initiate_connection();
    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);
    virtual void on_connect();

    // main loop begin
    void initiate_receive();
    void receive_callback(const boost::system::error_code err,
                          std::size_t receivedByteCount);
    
    // This is called regardless of the content of the message.
    // To be reimplemented in a subclass (does nothing by default).
    virtual void handle_message(const OculusMessageHeader& header,
                                const std::vector<uint8_t>& data);
};

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_SONAR_CLIENT_H_
