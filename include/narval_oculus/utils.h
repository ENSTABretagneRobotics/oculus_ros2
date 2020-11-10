#ifndef _DEF_NARVAL_OCULUS_UTILS_H_
#define _DEF_NARVAL_OCULUS_UTILS_H_

#include <sstream>

#include <narval_oculus/Oculus.h>

namespace narval { namespace oculus {

inline std::string ip_to_string(uint32_t ip)
{
    std::ostringstream oss;
    oss <<  (ip & 0x000000ff)        << "."
        << ((ip & 0x0000ff00) >>  8) << "."
        << ((ip & 0x00ff0000) >> 16) << "."
        << ((ip & 0xff000000) >> 24);
    return oss.str();
}


}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_UTILS_H_
