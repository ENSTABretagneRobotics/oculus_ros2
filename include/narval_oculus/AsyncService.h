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

#ifndef _DEF_NARVAL_OCULUS_ASYNC_SERVICE_H_
#define _DEF_NARVAL_OCULUS_ASYNC_SERVICE_H_

#include <iostream>
#include <thread>
#include <memory>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace narval { namespace oculus {

class AsyncService
{
    public:

    using IoService    = boost::asio::io_service;
    using IoServicePtr = std::shared_ptr<IoService>;

    protected:
    
    IoServicePtr service_;
    std::thread  thread_;
    bool         isRunning_;

    public:

    AsyncService();
    ~AsyncService();

    IoServicePtr io_service();

    bool is_running() const;
    void start();
    void stop();
};

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_ASYNC_SERVICE_H_
