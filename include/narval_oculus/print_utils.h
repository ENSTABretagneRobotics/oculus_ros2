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

#ifndef _DEF_NARVAL_OCULUS_PRINT_UTILS_H_
#define _DEF_NARVAL_OCULUS_PRINT_UTILS_H_

#include <iostream>
#include <sstream>

#include <narval_oculus/Oculus.h>

namespace narval { namespace oculus {

std::string ip_to_string(uint32_t ip);
std::string mac_to_string(const uint8_t* mac);

std::string to_string(DataSizeType dataType);
std::string to_string(PingRateType pingRate);
std::string to_string(OculusPartNumberType partNumber);

std::string to_string(const OculusMessageHeader& msg,     const std::string& prefix = "\n- ");
std::string to_string(const OculusStatusMsg& msg,         const std::string& prefix = "\n- ");
std::string to_string(const OculusSimpleFireMessage& msg, const std::string& prefix = "\n- ");
std::string to_string(const OculusSimplePingResult& msg,  const std::string& prefix = "\n- ");

}; //namespace oculus
}; //namespace narval

std::ostream& operator<<(std::ostream& os, DataSizeType dataType);
std::ostream& operator<<(std::ostream& os, PingRateType pingRate);
std::ostream& operator<<(std::ostream& os, OculusPartNumberType partNumber);

std::ostream& operator<<(std::ostream& os, const OculusMessageHeader& msg);
std::ostream& operator<<(std::ostream& os, const OculusStatusMsg& msg);
std::ostream& operator<<(std::ostream& os, const OculusSimpleFireMessage& msg);
std::ostream& operator<<(std::ostream& os, const OculusSimplePingResult& msg);

#endif //_DEF_NARVAL_OCULUS_PRINT_UTILS_H_
