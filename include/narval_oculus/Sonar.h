#ifndef _DEF_NARVAL_OCULUS_SONAR_H_
#define _DEF_NARVAL_OCULUS_SONAR_H_

#include <iostream>

#include <narval_oculus/AsyncService.h>
#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

// this is a convenience class 
class Sonar : public AsyncService, public SonarClient
{
    public:

    Sonar() :
        AsyncService(),
        SonarClient(this->ioService_)
    {}
};

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_SONAR_H_
