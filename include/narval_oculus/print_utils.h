#ifndef _DEF_NARVAL_OCULUS_PRINT_UTILS_H_
#define _DEF_NARVAL_OCULUS_PRINT_UTILS_H_

#include <iostream>
#include <sstream>

#include <narval_oculus/Oculus.h>

std::ostream& operator<<(std::ostream& os, const OculusMessageHeader& msg);
std::ostream& operator<<(std::ostream& os, const OculusStatusMsg& msg);

#endif //_DEF_NARVAL_OCULUS_PRINT_UTILS_H_
