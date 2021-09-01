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
