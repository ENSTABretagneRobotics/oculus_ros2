#ifndef _DEF_NARVAL_OCULUS_ASYNC_SERVICE_H_
#define _DEF_NARVAL_OCULUS_ASYNC_SERVICE_H_

#include <iostream>
#include <thread>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace narval { namespace oculus {

class AsyncService
{
    protected:
    
    boost::asio::io_service ioService_;
    std::thread             thread_;
    bool                    isRunning_;

    public:

    AsyncService();
    ~AsyncService();

    bool is_running() const;
    void start();
    void stop();
};

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_ASYNC_SERVICE_H_
