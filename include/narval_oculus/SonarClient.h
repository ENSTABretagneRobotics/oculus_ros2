/******************************************************************************
 * narval_oculus driver library for Blueprint Subsea Oculus sonar.
 * Copyright (C) 2020 ENSTA-Bretagne
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef _NARVAL_OCULUS_SONAR_CLIENT_H_
#define _NARVAL_OCULUS_SONAR_CLIENT_H_

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <type_traits>

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
 *
 * About concurrency on the socket : socket creation, destruction and read all
 * append in the same thread. There is no need to protect the socket for
 * concurrency between these operations. Also, boost sockets allows to be
 * concurrently read and written to at the same time. The situation where a
 * protection is needed is concurrent write and creation/destruction on the
 * socket. Hence, the socket is only locked in the send(), close_connection()
 * and ???. 
 */
template <typename TimeSourceT = std::chrono::system_clock>
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

    using TimeSource = TimeSourceT;
    using TimePoint  = typename std::invoke_result<decltype(&TimeSource::now)>::type;

    protected:
    
    IoServicePtr       ioService_;
    SocketPtr          socket_;
    EndPoint           remote_;
    uint16_t           sonarId_;
    ConnectionState    connectionState_;
    mutable std::mutex socketMutex_;

    Duration                     checkerPeriod_;
    boost::asio::deadline_timer  checkerTimer_;
    Clock                        clock_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;

    OculusMessageHeader    initialHeader_;
    std::vector<uint8_t>   data_;

    TimePoint recvTime_;

    // helper stubs
    void checker_callback(const boost::system::error_code& err);
    void check_reception(const boost::system::error_code& err);

    public:

    SonarClient(const IoServicePtr& ioService,
                const Duration& checkerPeriod = boost::posix_time::seconds(1));

    bool is_valid(const OculusMessageHeader& header);
    bool connected() const;

    size_t send(const boost::asio::streambuf& buffer) const;

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

    template <typename TimeT = float>
    TimeT time_since_last_message() const { return clock_.now<TimeT>(); }

    TimePoint last_header_stamp() const { return recvTime_; }
};

template <typename T>
SonarClient<T>::SonarClient(const IoServicePtr& service,
                         const Duration& checkerPeriod) :
    ioService_(service),
    socket_(nullptr),
    remote_(),
    sonarId_(0),
    connectionState_(Initializing),
    checkerPeriod_(checkerPeriod),
    checkerTimer_(*service, checkerPeriod_),
    statusListener_(service),
    statusCallbackId_(0),
    data_(0)
{
    std::memset(&initialHeader_, 0, sizeof(initialHeader_));

    this->checkerTimer_.async_wait(
        std::bind(&SonarClient<T>::checker_callback, this, std::placeholders::_1));
    this->reset_connection();
}

template <typename T>
bool SonarClient<T>::is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == OCULUS_CHECK_ID && header.srcDeviceId == sonarId_;
}

template <typename T>
bool SonarClient<T>::connected() const
{
    return connectionState_ == Connected;
}

template <typename T>
size_t SonarClient<T>::send(const boost::asio::streambuf& buffer) const
{
    std::unique_lock<std::mutex> lock(socketMutex_); // auto lock

    if(!socket_ || !this->connected())
        return 0;
    return socket_->send(buffer.data());
}

/**
 * Connection Watchdog.
 *
 * This is an asynchronous loop independent from the connection status (it is
 * looping even if there is no connection). It checks the status of the
 * connection with the sonar and restarts it if necessary.
 */
template <typename T>
void SonarClient<T>::checker_callback(const boost::system::error_code& err)
{
    // Programming now the next check 
    this->checkerTimer_.expires_from_now(checkerPeriod_);
    this->checkerTimer_.async_wait(
        std::bind(&SonarClient<T>::checker_callback, this, std::placeholders::_1));

    if(connectionState_ == Initializing || connectionState_ == Attempt) {
        // Nothing more to be done. Waiting.
        return;
    }
    
    auto lastStatusTime = statusListener_.time_since_last_status();
    if(lastStatusTime > 5) {
        // The status is retrieved through broadcasted UDP packets. No status
        // means no sonar on the network -> no chance to connect.
        // Still doing nothing because it might be a recoverable connection
        // loss.
        connectionState_ = Lost;
        std::cerr << std::setprecision(3) << "Connection lost for "
                  << lastStatusTime << "s\n";
        return;
    }

    if(this->time_since_last_message() > 10) {
        // Here last status was received less than 5 seconds ago but the last
        // message is more than 10s old. The connection is probably broken and
        // needs a reset.
        std::cerr << "Broken connection. Resetting.\n";
        this->reset_connection();
        return;
    }
}

template <typename T>
void SonarClient<T>::check_reception(const boost::system::error_code& err)
{
    // no real handling for now
    if(err) {
        std::ostringstream oss;
        oss << "oculus::SonarClient, reception error : " << err;
        //throw oss.str();
        std::cerr << oss.str() << std::endl;
    }
}

template <typename T>
void SonarClient<T>::reset_connection()
{
    connectionState_ = Attempt;
    this->close_connection(); // closing previous connection
    statusCallbackId_ = statusListener_.add_callback(&SonarClient<T>::on_first_status, this);
}

template <typename T>
void SonarClient<T>::close_connection()
{
    if(socket_) {
        std::unique_lock<std::mutex> lock(socketMutex_);
        std::cout << "Closing connection" << std::endl;
        try {
            boost::system::error_code err;
            socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, err);
            if(err) {
                std::cerr << "Error closing socket : '" << err << "'\n";
                return;
            }
            socket_->close();
        }
        catch(const std::exception& e) {
            std::cerr << "Error closing connection : " << e.what() << std::endl;
        }
        socket_ = nullptr;
    }
}

template <typename T>
void SonarClient<T>::on_first_status(const OculusStatusMsg& msg)
{
    // got a status message. No need to keep listening.
    statusListener_.remove_callback(statusCallbackId_);
    
    // device id and ip fetched from status message
    sonarId_ = msg.hdr.srcDeviceId;
    remote_ = remote_from_status<EndPoint>(msg);
    
    std::cout << "Got Oculus status"
              << "\n- netip   : " << ip_to_string(msg.ipAddr)
              << "\n- netmask : " << ip_to_string(msg.ipMask) << std::endl;

    // attempting connection
    socket_ = std::make_unique<Socket>(*ioService_);
    socket_->async_connect(remote_, boost::bind(&SonarClient<T>::on_connect, this, _1));
}

template <typename T>
void SonarClient<T>::on_connect(const boost::system::error_code& err)
{
    if(err) {
        std::ostringstream oss;
        oss << "oculus::SonarClient : connection failure. ( " << remote_ << ")";
        throw std::runtime_error(oss.str());
    }
    std::cout << "Connection successful (" << remote_ << ")" << std::endl << std::flush;
    
    clock_.reset();

    connectionState_ = Connected;
    // this enters the ping data reception loop
    this->initiate_receive();

    this->on_connect();
}

template <typename T>
void SonarClient<T>::on_connect()
{
    // To be reimplemented in a subclass
}

template <typename T>
void SonarClient<T>::initiate_receive()
{
    if(!socket_) return;
    // asynchronously scan input until finding a valid header.
    // /!\ To be checked : This function and its callback handle the data
    // synchronization with the start of the ping message. It is relying on the
    // assumption that if the data left in the socket is less than the size of
    // the header, a short read will happen, so that the next read will be
    // exactly aligned on the next header.
    static unsigned int count = 0;
    //std::cout << "Initiate receive : " << count << std::endl << std::flush;
    count++;
    boost::asio::async_read(*socket_,
        boost::asio::buffer(reinterpret_cast<uint8_t*>(&initialHeader_), 
                            sizeof(initialHeader_)),
        boost::bind(&SonarClient<T>::header_received_callback, this, _1, _2));
}

template <typename T>
void SonarClient<T>::header_received_callback(const boost::system::error_code err,
                                           std::size_t receivedByteCount)
{
    static unsigned int count = 0;
    //std::cout << "Receive callback : " << count << std::endl << std::flush;
    count++;
    // This function received only enough bytes for an OculusMessageHeader.  If
    // the header is valid, the control is dispatched to the next state
    // depending on the header content (message type). For now only simple ping
    // is implemented, but it seems to be the only message sent by the Oculus.
    // (TODO : check this last statement. Checked : wrong. Other message types
    // seem to be sent but are not documented by Oculus).
    this->check_reception(err);
    if(receivedByteCount != sizeof(initialHeader_) || !this->is_valid(initialHeader_)) {
        // Either we got data in the middle of a ping or did not get enougth
        // bytes (end of message). Continue listening to get a valid header.
        std::cout << "Header reception error" << std::endl << std::flush;
        this->initiate_receive();
        return;
    }
    recvTime_ = TimeSource::now();

    // Messsage header is valid. Now getting the remaining part of the message.
    // (The header contains the payload size, we can receive everything and
    // parse afterwards).
    data_.resize(sizeof(initialHeader_) + initialHeader_.payloadSize);
    boost::asio::async_read(*socket_,
        boost::asio::buffer(data_.data() + sizeof(initialHeader_), initialHeader_.payloadSize),
        boost::bind(&SonarClient<T>::data_received_callback, this, _1, _2));
}

template <typename T>
void SonarClient<T>::data_received_callback(const boost::system::error_code err,
                                         std::size_t receivedByteCount)
{
    if(receivedByteCount != initialHeader_.payloadSize) {
        // We did not get enough bytes. Reinitiating reception.
        std::cout << "Data reception error" << std::endl << std::flush;
        this->initiate_receive();
        return;
    }

    // We did received everything. copying initial header in data_ to have a
    // full message, then dispatching.
    *(reinterpret_cast<OculusMessageHeader*>(data_.data())) = initialHeader_;
    
    clock_.reset();
    // handle message is to be reimplemented in a subclass
    this->handle_message(initialHeader_, data_);

    // Continuing the reception loop.
    //std::cout << "Looping" << std::endl << std::flush;
    this->initiate_receive();
}

template <typename T>
void SonarClient<T>::handle_message(const OculusMessageHeader& header,
                                 const std::vector<uint8_t>& data)
{
    // To be reimplemented in a subclass
}

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_SONAR_CLIENT_H_
