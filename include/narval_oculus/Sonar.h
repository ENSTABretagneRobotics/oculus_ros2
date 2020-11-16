#ifndef _DEF_NARVAL_OCULUS_SONAR_H_
#define _DEF_NARVAL_OCULUS_SONAR_H_

#include <iostream>
#include <thread>

#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

// this is a convenience class 
class Sonar : public SonarClient
{
    protected:
    
    boost::asio::io_service ioService_;
    std::thread             thread_;
    bool                    isRunning_;

    public:

    Sonar();
    ~Sonar();

    bool is_running() const;
    void start();
    void stop();
};

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_SONAR_H_
