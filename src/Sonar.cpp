#include <narval_oculus/Sonar.h>

namespace narval { namespace oculus {

Sonar::Sonar() :
    SonarClient(ioService_),
    isRunning_(false)
{}

Sonar::~Sonar()
{
    this->stop();
}

bool Sonar::is_running() const
{
    return isRunning_;
}

void Sonar::start()
{
    if(this->is_running()) return;

    if(ioService_.stopped())
        ioService_.reset();

    thread_ = std::thread(boost::bind(&boost::asio::io_service::run, &ioService_));
    if(!thread_.joinable())
        throw std::runtime_error("Failed to start Sonar");

    isRunning_ = true;
}

void Sonar::stop()
{
    if(!this->is_running()) return;

    std::cout << "stopping" << std::endl;
    
    ioService_.stop();
    thread_.join();
    if(thread_.joinable())
        throw std::runtime_error("Failed to stop Sonar");

    isRunning_ = false;

    std::cout << "stopped" << std::endl;
}

}; //namespace oculus
}; //namespace narval
