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

#ifndef _DEF_NARVAL_OCULUS_STATUS_LISTENER_H_
#define _DEF_NARVAL_OCULUS_STATUS_LISTENER_H_

#include <iostream>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/CallbackQueue.h>
#include <narval_oculus/Clock.h>

namespace narval { namespace oculus {

class StatusListener
{
    public:

    using IoService    = boost::asio::io_service;
    using IoServicePtr = std::shared_ptr<IoService>;
    using Socket       = boost::asio::ip::udp::socket;
    using EndPoint     = boost::asio::ip::udp::endpoint;
    using Callbacks    = CallbackQueue<const OculusStatusMsg&>;
    using CallbackT    = Callbacks::CallbackT;
    using CallbackId   = Callbacks::CallbackId;

    protected:

    Socket          socket_;
    EndPoint        remote_;
    OculusStatusMsg msg_;
    Callbacks       callbacks_;
    Clock           clock_;

    public:

    StatusListener(const IoServicePtr& service, unsigned short listeningPort = 52102);
    
    template <typename F, class... Args>
    CallbackId add_callback(F&& func, Args&&... args);
    CallbackId add_callback(const CallbackT& callback);
    bool remove_callback(CallbackId index);
    template <typename F, class... Args>
    bool on_next_status(F&& func, Args&&... args);
    bool on_next_status(const CallbackT& callback);

    template <typename T = float>
    T time_since_last_status() const { return clock_.now<T>(); }
    
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

template <typename F, class... Args>
bool StatusListener::on_next_status(F&& func, Args&&... args)
{
    return this->on_next_status(static_cast<const CallbackT&>(
        std::bind(func, args..., std::placeholders::_1)));
}

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_STATUS_LISTENER_H_
