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

#include <narval_oculus/AsyncService.h>

namespace narval { namespace oculus {

AsyncService::AsyncService() :
    service_(std::make_unique<IoService>()),
    isRunning_(false)
{}

AsyncService::~AsyncService()
{
    this->stop();
}

AsyncService::IoServicePtr AsyncService::io_service()
{
    return service_;
}

bool AsyncService::is_running() const
{
    return isRunning_;
}

void AsyncService::start()
{
    if(this->is_running()) return;
    std::cout << "starting" << std::endl;

    if(service_->stopped())
        service_->reset();

    thread_ = std::thread(boost::bind(&boost::asio::io_service::run, service_));
    if(!thread_.joinable())
        throw std::runtime_error("Failed to start AsyncService");

    isRunning_ = true;
}

void AsyncService::stop()
{
    if(!this->is_running()) return;

    std::cout << "stopping" << std::endl;
    
    service_->stop();
    thread_.join();
    if(thread_.joinable())
        throw std::runtime_error("Failed to stop AsyncService");

    isRunning_ = false;

    std::cout << "stopped" << std::endl;
}

}; //namespace oculus
}; //namespace narval
